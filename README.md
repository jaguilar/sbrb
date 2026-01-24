# Arduino CLI Template

## Overview

This template is designed to be used to construct Arduino projects
that you will compile and flash via the CLI. It's an alternative to
Arduino IDE that makes it easier to version control projects and
program Arduino stuff in VSCode. We could do PlatformIO, yea, but
they don't support the most recent RP2040 or ESP32 cores. And we
could use ArduinoIDE, but then it's hard to get at your files,
and that's annoying.

Instead, you can clone this folder as a template, update config.mk
for your target, and get to work.

## Usage

There are two ways to get started:

* Start editing main.cpp immediately. When you do `make compile`,
  we will automatically generate a blank `.ino` file that matches
  the project directory's name.
* Create `DIRECTORY_NAME.ino`, delete `main.cpp`, and begin editing
  from there.

## Plan

* Automatically generate launch.json for debugging using 
  `arduino-cli debug --info --json`.
