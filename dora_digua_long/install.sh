
#!/usr/bin/env sh

set -eu

if [ -n "${GITHUB_ACTIONS-}" ]; then
  set -x
fi

# Check pipefail support in a subshell, ignore if unsupported
# shellcheck disable=SC3040
(set -o pipefail 2> /dev/null) && set -o pipefail

help() {
  cat <<'EOF'
Install a binary released on GitHub

USAGE:
    install.sh [options]

FLAGS:
    -h, --help      Display this message
    -f, --force     Force overwriting an existing binary

OPTIONS:
    --repo REPO     Github Repository to install the binary from  [default: dora-rs]
    --bin BIN       Name of the binary to install  [default: dora]
    --tag TAG       Tag (version) of the bin to install, defaults to latest release
    --to LOCATION   Where to install the binary [default: ~/.dora/bin]
    --target TARGET
EOF
}


say() {
  echo "install: $*" >&2
}

err() {
  if [ -n "${td-}" ]; then
    rm -rf "$td"
  fi

  say "error: $*"
  exit 1
}

need() {
  if ! command -v "$1" > /dev/null 2>&1; then
    err "need $1 (command not found)"
  fi
}



download() {
  url="$1"
  output="$2"

  if command -v curl > /dev/null; then
    curl --proto =https --tlsv1.2 -sSfL "$url" "-o$output"
  else
    wget --https-only --secure-protocol=TLSv1_2 --quiet "$url" "-O$output"
  fi
}

force=false
while test $# -gt 0; do
  case $1 in
    --help | -h)
      help
      exit 0
      ;;
    --repo)
      repo=$2
      shift
      ;;
    --bin)
      bin=$2
      shift
      ;;
    --tag)
      tag=$2
      shift
      ;;
    --target)
      target=$2
      shift
      ;;
    --to)
      dest=$2
      shift
      ;;
    *)
      say "error: unrecognized argument '$1'. Usage:"
      help
      exit 1
      ;;
  esac
  shift
done

if [ -z "${repo-}" ]; then
  repo="dora-rs"
fi

if [ -z "${bin-}" ]; then
  bin="dora"
fi

url=https://github.com/$repo/$bin
releases=$url/releases

command -v curl > /dev/null 2>&1 ||
  command -v wget > /dev/null 2>&1 ||
  err "need wget or curl (command not found)"

need mkdir
need mktemp

if [ -z "${tag-}" ]; then
  need grep
  need cut
fi

if [ -z "${target-}" ]; then
  need cut
fi

if [ -z "${dest-}" ]; then
  dest="$HOME/.dora/bin"
fi


if [ -z "${tag-}" ]; then
  tag=$(
    download https://api.github.com/repos/$repo/$bin/releases/latest - |
    grep tag_name |
    cut -d'"' -f4
  )
fi


if [ -z "${target-}" ]; then
  # bash compiled with MINGW (e.g. git-bash, used in github windows runners),
  # unhelpfully includes a version suffix in `uname -s` output, so handle that.
  # e.g. MINGW64_NT-10-0.19044
  kernel=$(uname -s | cut -d- -f1)
  uname_target="$(uname -m)-$kernel"

  case $uname_target in
    aarch64-Linux) target=aarch64-unknown-linux-musl;;
    arm64-Darwin) target=aarch64-apple-darwin;;
    armv7l-Linux) target=armv7-unknown-linux-musleabihf;;
    x86_64-Darwin) target=x86_64-apple-darwin;;
    x86_64-Linux) target=x86_64-unknown-linux-gnu;;
    *)
      # shellcheck disable=SC2016
      err 'Could not determine target from output of `uname -m`-`uname -s`, please use `--target`:' "$uname_target"
    ;;
  esac
fi

case $target in
  *) extension=zip; need unzip;;
esac

archive="$releases/download/$tag/$bin-$tag-$target.$extension"
say "Repository:  $url"
say "Bin:         $bin"
say "Tag:         $tag"
say "Target:      $target"
say "Destination: $dest"
say "Archive:     $archive"

td=$(mktemp -d || mktemp -d -t tmp)

if [ "$extension" = "zip" ]; then
  download "$archive" "$td/$bin.zip"
  unzip -d "$td" "$td/$bin.zip"
else
  download "$archive" - | tar -C "$td" -xz
fi

echo "Placing dora-rs cli in $dest"

if [ -e "$dest/$bin" ] && [ "$force" = false ]; then
  echo " Replacing \`$dest/$bin\` with downloaded version"
  cp "$td/$bin" "$dest/$bin"
  chmod 755 "$dest/$bin"
else
  mkdir -p "$dest"
  cp "$td/$bin" "$dest/$bin"
  chmod 755 "$dest/$bin"
  echo ""
fi

if [ "$SHELL" = "/bin/bash" ]; then
    if ! grep -q "$dest" ~/.bashrc; then
        echo "Adding $dest to PATH in ~/.bashrc"
        echo "export PATH=\$PATH:$dest" >> ~/.bashrc
        echo "Path added to ~/.bashrc."
        echo "Please reload with:"
        echo "  source ~/.bashrc"
    else
        echo "$dest is already in the PATH in ~/.bashrc"
    fi
elif [ "$SHELL" = "/bin/zsh" ]; then
    if ! grep -q "$dest" ~/.zshrc; then
        echo "Adding $dest to PATH in ~/.zshrc"
        echo "export PATH=\$PATH:$dest" >> ~/.zshrc
        echo "Path added to ~/.zshrc."
        echo "Please reload with:"
        echo "  source ~/.zshrc"
    else
        echo "$dest is already in the PATH in ~/.zshrc"
    fi
else
    echo "Unsupported shell: $SHELL"
    echo "Please add the following to your shell's configuration file manually:"
    echo "    export PATH=\$PATH:$dest"
fi


rm -rf "$td"
