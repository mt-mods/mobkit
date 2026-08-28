# Mobkit

[![luacheck](https://github.com/mt-mods/mobkit/actions/workflows/luacheck.yml/badge.svg)](https://github.com/mt-mods/mobkit/actions/workflows/luacheck.yml)

Entity API for Luanti/Minetest

This library is meant to be shared between mods.
Please do not write to the mobkit namespace ('mobkit' global table),
nor include own copies of mobkit in your mods and modpacks.
Instead in `mod.conf` place `depends = modkit` to use this library.

## Usage

Please refer to [API.md](/API.md).

## Dependencies

- Luanti/Minetest v5.0
