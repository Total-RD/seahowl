#!/bin/bash

# List of extensions to install
extensions=(
"matepek.vscode-catch2-test-adapter"
"ms-python.debugpy"
"ms-python.python"
"ms-python.vscode-pylance"
"ms-vscode.cmake-tools"
"ms-vscode.cpptools"
"ms-vscode.cpptools-extension-pack"
"ms-vscode.cpptools-themes"
"ms-vscode.live-server"
"twxs.cmake"
)

# Install each extension
for extension in "${extensions[@]}"; do
    code --install-extension "$extension"
done
