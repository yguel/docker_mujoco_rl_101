#!/bin/bash
# Quick launcher script for MuJoCo environment

# Function to show menu using zenity
show_menu() {
    choice=$(zenity --list \
        --title="MuJoCo Quick Launcher" \
        --text="Choose an application to launch:" \
        --column="Application" \
        --column="Description" \
        --width=500 \
        --height=300 \
        "Visual Studio Code" "Code editor with MuJoCo examples" \
        "Google Chrome" "Web browser for documentation" \
        "Terminal" "Command line interface" \
        "File Manager" "Browse workspace files" \
        "MuJoCo Examples" "Open examples in VS Code" \
        2>/dev/null)

    case "$choice" in
        "Visual Studio Code")
            code --no-sandbox --disable-gpu --user-data-dir=/home/student/.vscode /home/student/workspace &
            ;;
        "Google Chrome")
            google-chrome --no-sandbox --disable-gpu &
            ;;
        "Terminal")
            mate-terminal &
            ;;
        "File Manager")
            caja /home/student/workspace &
            ;;
        "MuJoCo Examples")
            code --no-sandbox --disable-gpu --user-data-dir=/home/student/.vscode /home/student/workspace/examples &
            ;;
    esac
}

show_menu