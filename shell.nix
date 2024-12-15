let
	nixpkgs = fetchTarball "https://github.com/NixOS/nixpkgs/tarball/nixos-unstable";
	pkgs = import nixpkgs { config = {}; overlays = []; };
in

pkgs.mkShell {
	packages = with pkgs; [
		pkg-config

		# Tools
		lldb

		clang-tools
		clang
		bear
	];

	shellHook = ''
	$(cat /etc/passwd | grep $(whoami) | cut -d':' -f7); exit $?
	'';
}
