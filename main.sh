DIR="$(dirname "${BASH_SOURCE[0]}")"

gnome-terminal "main" -- bash -c "cd $DIR;bash start.sh bash"

