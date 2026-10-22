# Progress captures

These image files are actual captures of openASC rendering the user's local
Assetto Corsa installation, approved for the public project website.

- `first-drive-start.jpg`: BMW M3 E30 at Drift AC_PIT_0, initially paused.
- `first-drive-moving.jpg`: the keyboard prototype moving with four ROAD contacts.
- `first-drive-wall.jpg`: stopped after static WALL contact, shown from an orbit view.
- `native-drift.png`: first Rust/wgpu diagnostic geometry frame, Vulkan/RTX 2080, without textures or original material states.
- `native-window-drift.png`: 2026-10-18 perspective GPU readback from the native Windows/Vulkan surface, without window decorations.
- `native-magione-composed.png`: 2026-10-18 Rust models.ini composition of two Magione KN5 sources, with 622 draws. Eight resources exceeded the shared decoded-texture budget and use diagnostic colours; road/grass materials are visibly incomplete.
- `native-magione-bc.png`: subsequent 2026-10-18 Vulkan/RTX 2080 frame with all 44 referenced diffuse textures uploaded directly as BC data, restoring asphalt within the bounded payload budget. Original grass/shader behavior remains incomplete.
- `native-drift-multilayer.png`: subsequent 2026-10-18 frame with 46 masked four-detail base-colour draws and source grass colour; shading/stippling/coplanar behavior remains approximate.
- `native-magione-multilayer.png`: corresponding two-model Magione frame with 85 multilayer draws and 63 resources; full original lighting/material parity remains unvalidated.
- `native-drift-textured.png`: subsequent native diffuse frame with 25 embedded textures and approximate alpha/depth states. White/stippled grass reflects unsupported original material behavior.

Captured on 2026-10-17 and 2026-10-18. No source models, textures or other installation files are
distributed with these images. They document the approximate prototype rather
than original-simulator visual or physics parity.

The images depict Assetto Corsa content. The project's MIT license does not
relicense that underlying content. Original project code and vector branding
remain covered by their stated MIT license.

Add representative captures after future visual milestones. Keep diagnostic
reports, temporary captures and source game assets in ignored local directories;
only selected website illustrations belong here.


2026-10-22 native driving milestone:

- `native-drive-start.png`: 800 x 600 GPU surface readback from the Rust/wgpu BMW M3 E30/Drift driving window at time zero.
- `native-drive-turn.png`: the same fixed camera at 1.6 simulated seconds, after throttle and rack steering through the diagnostic shared native key handler. The actual coupled chassis pose drives the car root; wheel meshes are not individually animated. Synthetic masses/mounts/forces and explicit visual COM mapping remain uncalibrated against the original game.

These selected flattened images contain no reusable source models or texture files. Their underlying game-content rights are unchanged.
