struct Camera { view_projection: mat4x4<f32>, model: mat4x4<f32> };
struct Material { flags: vec4<f32>, lighting: vec4<f32>, layers:vec4<f32>, uv_rg:vec4<f32>, uv_ba:vec4<f32> };
@group(0) @binding(0) var<uniform> camera: Camera;
@group(1) @binding(0) var diffuse_texture: texture_2d<f32>;
@group(1) @binding(1) var diffuse_sampler: sampler;
@group(1) @binding(2) var<uniform> material: Material;
@group(1) @binding(3) var layer_mask: texture_2d<f32>;
@group(1) @binding(4) var detail_r: texture_2d<f32>;
@group(1) @binding(5) var detail_g: texture_2d<f32>;
@group(1) @binding(6) var detail_b: texture_2d<f32>;
@group(1) @binding(7) var detail_a: texture_2d<f32>;
struct Surface {
    @builtin(position) clip: vec4<f32>,
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) colour: vec3<f32>,
};
@vertex fn vertex(@location(0) position: vec3<f32>, @location(1) normal: vec3<f32>, @location(2) uv: vec2<f32>, @location(3) colour: vec3<f32>) -> Surface {
    var out: Surface;
    let world = camera.model * vec4<f32>(position, 1.0);
    out.clip = camera.view_projection * world;
    out.position = world.xyz;
    out.normal = (camera.model * vec4<f32>(normal, 0.0)).xyz;
    out.uv = uv;
    out.colour = colour;
    return out;
}
@fragment fn fragment(surface: Surface) -> @location(0) vec4<f32> {
    let texel = textureSample(diffuse_texture, diffuse_sampler, surface.uv);
    let geometric = cross(dpdx(surface.position), dpdy(surface.position));
    var normal = surface.normal;
    if length(normal) < 0.000001 { normal = geometric; }
    normal = normal / max(length(normal), 0.000001);
    var base = surface.colour;
    if material.flags.x > 0.5 { base = texel.rgb; }
    if material.layers.x > 0.5 {
        let coords = select(surface.uv, surface.position.xz, material.layers.y > 0.5);
        let weights = textureSample(layer_mask,diffuse_sampler,surface.uv);
        let r = textureSample(detail_r,diffuse_sampler,coords*material.uv_rg.xy).rgb;
        let g = textureSample(detail_g,diffuse_sampler,coords*material.uv_rg.zw).rgb;
        let b = textureSample(detail_b,diffuse_sampler,coords*material.uv_ba.xy).rgb;
        let a = textureSample(detail_a,diffuse_sampler,coords*material.uv_ba.zw).rgb;
        base *= (weights.r*r + weights.g*g + weights.b*b + weights.a*a)*material.layers.z;
    }
    var alpha = 1.0;
    if material.flags.z > 0.5 { alpha = texel.a * material.flags.w; }
    if alpha < material.flags.y || alpha < 0.001 { discard; }
    let light = (0.35 + 0.65 * abs(dot(normal, normalize(vec3<f32>(0.4, 1.0, 0.3))))) * material.lighting.x;
    return vec4<f32>(base * light, alpha);
}
