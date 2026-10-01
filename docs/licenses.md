# Licenses and redistribution scope

## Distributed files

| Material | License | Attribution |
|---|---|---|
| Project-authored code, documentation and configuration, unless noted below | [BSD-3-Clause](../LICENSE.txt) | rosotacom contributors; headers identify authors and dates |
| Three ROS templates in `src/rosotacom/resources/ws/ros2src/com_py/test/` | [Apache-2.0](../LICENSES/Apache-2.0.txt) | Open Source Robotics Foundation, Inc., 2015 and 2017; notices retained |
| `netem_profiles.yaml`, `replay/r3_frame_plan.csv`, `transit/r6c.jsonl.gz` under `field-profiles/2026-08/` | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/legalcode.en) | Martin Gontscharow, FZI Research Center for Information Technology, 2026 |

The [distribution notice](../NOTICE.txt) accompanies packaged releases.

The Apache files are `test_copyright.py`, `test_flake8.py` and `test_pep257.py`.
They remain Apache-licensed, not relicensed as BSD. Their license and notices
accompany the Python distribution. The package license expression includes
these exceptions and the field data in its source distribution. The top-level
BSD license does not relicense third-party files or data.

Data attribution, with the release URL: "Martin Gontscharow, FZI Research
Center for Information Technology (2026), Fitted network profiles, frame plan
and L6 transit metadata, CC BY 4.0." Retain the license link and indicate
changes when adapting data. FZI-RECAP has separate license information.

New header dates identify publication metadata where no original creation
date was recorded. Original names and dates are retained, including Daniel
Bogdoll's attribution in the Wi-Fi utility.

## Dependencies retain their licenses

Installation resolves separate packages. This repository does not vendor the
implementations below or distribute a fully built container.

| Direct host dependency | License of inspected package |
|---|---|
| argcomplete 3.7.2 | Apache-2.0 |
| mcap 1.5.0 | MIT |
| Pillow 12.3.0 | MIT-CMU; binary wheels carry additional notices |
| PyYAML 6.0.3 | MIT |
| ros2docker 0.1.5.dev10 | BSD-3-Clause |
| matplotlib, optional plots | Matplotlib/PSF-based license, see its installed license |

These inspected versions are not a dependency lock. Future versions and
transitive binary contents need separate review. The ROS runtime also uses
ROS 2 packages, DDS/Zenoh, image-transport plugins, compression libraries and
system utilities. Their notices remain applicable.

Primary sources and redistribution boundaries:

- [BSD-3-Clause](https://opensource.org/license/bsd-3-clause): retain its
  copyright notice, conditions and disclaimer.
- [Apache-2.0, section 4](https://www.apache.org/licenses/LICENSE-2.0): include
  the license and applicable notices when redistributing covered files.
  [rclpy](https://github.com/ros2/rclpy/blob/rolling/LICENSE) also uses Apache-2.0.
- [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/LICENSE)
  offers EPL-2.0 or EDL-1.0. Retain the selected distribution's notices.
- [FFmpeg](https://ffmpeg.org/legal.html) is LGPL-2.1-or-later with GPL parts
  depending on its build; libx264 is GPL-licensed. A container with these is
  not an all-BSD binary distribution. Redistributing it requires handling the
  actual build's notices and corresponding-source obligations. Codec patent
  questions are separate.

The source review found no basis for removing the project's BSD-3-Clause
license. It does not certify every container build, prove the origin of every
historical line, or establish an employer's publication approval.
