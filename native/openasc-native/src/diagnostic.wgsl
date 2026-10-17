struct Camera { view_projection: mat4x4<f32> };
@group(0) @binding(0) var<uniform> camera: Camera;
struct Surface {
    @builtin(position) clip: vec4<f32>,
    @location(0) position: vec3<f32>,
    @location(1) colour: vec3<f32>,
};
@vertex fn vertex(@location(0) position: vec3<f32>, @location(1) colour: vec3<f32>) -> Surface {
    var out: Surface;
    out.clip = camera.view_projection * vec4<f32>(position, 1.0);
    out.position = position;
    out.colour = colour;
    return out;
}
@fragment fn fragment(surface: Surface) -> @location(0) vec4<f32> {
    let n = cross(dpdx(surface.position), dpdy(surface.position));
    let normal = n / max(length(n), 0.000001);
    let light = 0.35 + 0.65 * abs(dot(normal, normalize(vec3<f32>(0.4, 1.0, 0.3))));
    return vec4<f32>(surface.colour * light, 1.0);
}
