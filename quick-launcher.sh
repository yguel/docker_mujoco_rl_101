#!/bin/bash
# Quick launcher script for MuJoCo environment

# Function to show menu using zenity
show_menu() {
    choice=$(zenity --list \
        --title="Quick Launcher for Tools" \
        --text="Choose an application to launch:" \
        --column="Application" \
        --column="Description" \
        --width=500 \
        --height=440 \
        "Visual Studio Code" "Code editor" \
        "Chromium" "Web browser for documentation" \
        "Gedit" "Text editor" \
        "Terminal" "Command line interface" \
        "File Manager" "Browse workspace files" \
        "Image Viewer" "View images and SVG files" \
        "PDF Viewer" "View PDF documents")

    case "$choice" in
        "Visual Studio Code, open MuJoCo workspace")
            code --no-sandbox --disable-gpu --user-data-dir=/home/student/.vscode /home/student/workspace &
            ;;
        "Chromium")
            chromium-browser --no-sandbox --disable-gpu &
            ;;
        "Gedit")
            gedit &
            ;;
        "Terminal")
            mate-terminal &
            ;;
        "File Manager")
            caja /home/student/workspace &
            ;;
        "Image Viewer")
            eog &
            ;;
        "PDF Viewer")
            evince &
            ;;
    esac
}

show_menu