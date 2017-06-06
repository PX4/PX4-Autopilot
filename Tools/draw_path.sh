#!/bin/sh
#
# Script to draw 2D flight paths. The z-component is always 0.
# The purpose is to test the flight graph tracker by handing it a predesigned path.
#
# Usage: draw_path.sh [file]
# If the file already exists, new points are appended.
# arrow keys:   move cursor
# spacebar:     add current cursor position to path
# q:            quit



# Shows a status message at the end of the screen
function show_status() {
    tput cup $(expr $SCREEN_WIDTH-1) 0
    echo $1
    move_cursor
    STATUS_CLEAR="no"
}

# Clears the status message
function clear_status() {
    [ "$STATUS_CLEAR" = "no" ] || return 0
    tput cup 0 0
    tput el && tput el1
    move_cursor
    STATUS_CLEAR="yes"
}

# Terminates the script
function quit() {
    tput rmcup
    stty echo
    exit 0
}

# Moves the terminal cursor to position ($CURSOR_X, $CURSOR_Y)
function move_cursor() {
    #echo "\033[$CURSOR_Y;$CURSOR_X"'f'
    #echo -e "\033[$CURSOR_Y;3f"
    tput cup $CURSOR_Y $CURSOR_X
}

# Draws a line from the last point to the current cursor position
function go_here() {
    draw_line $LAST_POINT_X $LAST_POINT_Y $CURSOR_X $CURSOR_Y
    echo "o"
    move_cursor
    LAST_POINT_X=$CURSOR_X
    LAST_POINT_Y=$CURSOR_Y
}

# Adds the current cursor position to the file $FILE
function store_point() {
    echo "$CURSOR_X,$CURSOR_Y,0," >> $FILE
}

# Loads all points from the file $FILE
# Each line stores a point in the format "x,y,z".
# Empty lines and lines starting with "/" are ignored.
function load_file() {
    while IFS='' read -r line || [[ -n "$line" ]]; do
        CURSOR_X=$(echo "$line" | awk -F, "/^[^\/]/{print \$1}")
        CURSOR_Y=$(echo "$line" | awk -F, "/^[^\/]/{print \$2}")
        #echo "X=$CURSOR_X Y=$CURSOR_Y"
	[ "$CURSOR_X" = "" ] || [ "$CURSOR_Y" = "" ] ||	go_here
    done < "$FILE"
}

# Draws a line using the Bresenham algorithm
# usage: draw_line x1 y1 x2 y2
# The point (x1, y1) is not filled.
# The point (x2, y2) is filled and the cursor is placed there.
function draw_line() {
    X1=$1
    Y1=$2
    X2=$3
    Y2=$4
    LINE_CHAR="\\"

    # The algorithm requires that delta-y is smaller than delta-x
    SWAP_XY=$(echo "define delta(a, b) {if (a>b) return a-b; return b-a;}; delta($X1,$X2) < delta($Y1,$Y2)" | bc)
    [ "$SWAP_XY" = "1" ] && {
        TEMP=$X1; X1=$Y1; Y1=$TEMP
        TEMP=$X2; X2=$Y2; Y2=$TEMP
    }

    # Delta-x must be positive
    REVERSE_DIR=$(echo "$X1 > $X2" | bc)
    [ "$REVERSE_DIR" = "1" ] && {
        TEMP=$X1; X1=$X2; X2=$TEMP
        TEMP=$Y1; Y1=$Y2; Y2=$TEMP
    }

    # Now the slope is in [-45°, +45], in positive x-direction. The update update differs for positive and negative slopes.
    POS_SLOPE=$(echo "$Y2 > $Y1" | bc)

    DELTA_X=$(($X2-$X1))
    DELTA_Y=$(($Y2-$Y1))

    # init update criterion
    if [ "$POS_SLOPE" = "1" ]; then
        D=$(echo "$DELTA_Y * ($X1+1) - $DELTA_X * ($Y1+0.5) + $X2 * $Y1 - $X1 * $Y2" | bc)
    else
        D=$(echo "$DELTA_Y * ($X1+1) - $DELTA_X * ($Y1-0.5) + $X2 * $Y1 - $X1 * $Y2" | bc)
    fi

    XP=$X1
    YP=$Y1

    while [ $XP -lt $X2 ]; do
        [ "$SWAP_XY" = "0" ] && LINE_CHAR="-" || LINE_CHAR="|"

        # Move to next pixel, according to update criterion
        XP=$(($XP+1))
        if [ "$POS_SLOPE" = "1" ]; then
            [ $(echo "$D > 0" | bc) = "1" ] && {
                YP=$(($YP+1))
                D=$(echo "$D - $DELTA_X" | bc)
                LINE_CHAR="\\"
            }
            D=$(echo "$D + $DELTA_Y" | bc)
        else
            [ $(echo "$D < 0" | bc) = "1" ] && {
                YP=$(($YP-1))
                D=$(echo "$D + $DELTA_X" | bc)
                LINE_CHAR="/"
            }
            D=$(echo "$D + $DELTA_Y" | bc)
        fi

        # Draw pixel
        [ "$SWAP_XY" = "0" ] && { CURSOR_X=$XP; CURSOR_Y=$YP; } || { CURSOR_X=$YP; CURSOR_Y=$XP; } 
        move_cursor;

        ([ "$REVERSE_DIR" = "1" ] && [ "$XP" = "$X2" ]) || echo "$LINE_CHAR"
    done
    
    [ "$REVERSE_DIR" = "1" ] && { [ "$SWAP_XY" = "0" ] && { CURSOR_X=$X1; CURSOR_Y=$Y1; } || { CURSOR_X=$Y1; CURSOR_Y=$X1; } }
    move_cursor;
}

# Moves the cursor up if possible
function up() {
    [ $CURSOR_Y -gt 0 ] && let CURSOR_Y=$CURSOR_Y-1
    move_cursor
}

# Moves the cursor down if possible
function down() {
    let CURSOR_Y=$CURSOR_Y+1
    [ $CURSOR_Y -lt $SCREEN_HEIGHT ] || let CURSOR_Y=$SCREEN_HEIGHT-1
    move_cursor
}

# Moves the cursor left if possible
function left() {
    [ $CURSOR_X -gt 0 ] && let CURSOR_X=$CURSOR_X-1
    move_cursor
}

# Moves the cursor right if possible
function right() {
    let CURSOR_X=$CURSOR_X+1
    [ $CURSOR_X -lt $SCREEN_WIDTH ] || let CURSOR_X=$SCREEN_WIDTH-1
    move_cursor
}




# Set up globals
SCREEN_WIDTH=$(tput cols)
SCREEN_HEIGHT=$(tput lines)
let CURSOR_X=$SCREEN_WIDTH/2
let CURSOR_Y=$SCREEN_HEIGHT/2
LAST_POINT_X=$CURSOR_X
LAST_POINT_Y=$CURSOR_Y
KEY_SEQUENCE=""
FILE=$1

touch $FILE 2>/dev/null || { echo "could not open file $FILE"; exit 1; }

# Switch to alternate screen
tput smcup
#x=$(tput lines)
#while [ $x -gt 0 ]; do echo ""; let x=$x-1; done;
tput clear
stty -echo
move_cursor

load_file


# draws two crosses
#draw_line 20 3 50 15
#draw_line 20 15 50 3
#draw_line 5 5 20 30
#draw_line 20 5 5 30

# draws the same two crosses (with swapped start end end points)
#draw_line 50 15 20 3
#draw_line 50 3 20 15
#draw_line 20 30 5 5
#draw_line 5 30 20 5


while true; do
    IFS=""
    read -r -n 1 CHAR

    KEY_SEQUENCE=$KEY_SEQUENCE$CHAR
    CARRY_SEQUENCE=""

    clear_status

    case $KEY_SEQUENCE in
        $'\033'|$'\033[')
            # incomplete escape sequence - read another char
            CARRY_SEQUENCE=$KEY_SEQUENCE
            #show_status "press q to exit"
            ;;

        'q') quit ;;

        $'\033[A') up ;;
        $'\033[B') down ;;
        $'\033[C') right ;;
        $'\033[D') left ;;

        $" ") go_here; store_point ;;

        *)
            show_status "unknown key"
            ;;
    esac

    KEY_SEQUENCE=$CARRY_SEQUENCE
done

