#!/bin/bash

# Pre-requirements
# apt update && apt install curl build-essential

# Remove old versions
if dpkg -l | grep -q bindgen; then
    sudo dpkg --remove bindgen
fi
if dpkg -l | grep -q cargo; then
    sudo dpkg --remove cargo
fi
if dpkg -l | grep -q rustc; then
    sudo dpkg --remove rustc
fi


# Install Rust compiler
curl https://sh.rustup.rs -sSf | sh -s -- -y

# Source the cargo environment to update PATH
source "$HOME/.cargo/env"

rustup toolchain install stable-x86_64-unknown-linux-gnu
rustup target add aarch64-unknown-none riscv64gc-unknown-none-elf
cargo install --force bindgen-cli
