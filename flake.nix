{
  description = "simulator dev flake";
  inputs = {
    nixpkgs.url = "tarball+https://codeload.github.com/NixOS/nixpkgs/tar.gz/e2605d0744c2417b09f8bf850dfca42fcf537d34";
    flake-utils = {
      url = "https://codeload.github.com/numtide/flake-utils/tar.gz/11707dc2f618dd54ca8739b309ec4fc024de578b";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };
  outputs = { self, nixpkgs, flake-utils }:
  flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs {
        inherit system;
        config = {
          allowUnfree = true;
        };
      };
    in
      {
        devShell = pkgs.mkShell {
          buildInputs =  with pkgs; [
            clang-tools
            # boost
            (boost.override { 
              enableShared = false;
              enableStatic = true;
            })
            # (icu.override { 
            #   withStatic = true;
            # })

            (icu.overrideAttrs (oldAttrs: {
              withStatic = true;
            }))


            bear
            binutils
            cmake
            gnumake
          ];
        };
    });
}




