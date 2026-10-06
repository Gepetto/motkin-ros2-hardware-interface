{
  description = "ros2_control SystemInterface hardware plugin that drives a dual-motor testbed";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        rosDistros = [ "jazzy" ];
        rosShellDistro = "jazzy";
        rosOverrideAttrs.motkin-ros2-hardware-interface = {
          src = lib.fileset.toSource {
            root = ./.;
            fileset = lib.fileset.unions [
              ./CMakeLists.txt
              ./CONTRIBUTING.md
              ./include
              ./LICENSE
              ./motkin_ros2_hardware_interface.xml
              ./package.xml
              ./ros2_control
              ./src
            ];
          };
        };
      }
    );
}
