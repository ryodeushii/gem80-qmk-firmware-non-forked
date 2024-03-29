# NuPhy Gem80


* Keyboard Maintainer: [nuphy](https://github.com/nuphy-src)
* Hardware Supported: NuPhy Gem 80 Tri-Mode
* Hardware Availability: Private

Make example for this keyboard (after setting up your build environment):

    make nuphy/gem80/ansi:via

Flashing example for this keyboard:

    make nuphy/gem80/ansi:via:flash

See the [build environment setup](https://docs.qmk.fm/#/getting_started_build_tools) and the [make instructions](https://docs.qmk.fm/#/getting_started_make_guide) for more information. Brand new to QMK? Start with our [Complete Newbs Guide](https://docs.qmk.fm/#/newbs).

## Bootloader

Enter the bootloader in one way:

* **Bootmagic reset**: Hold down the key at (0,0) in the matrix (usually the top left key or Escape) and plug in the keyboard

* **Hardware reset**: Remove the capslock keycap, hold the little button beneath and plug in the keyboard.



## Battery indication (level to color mapping)

```
0-20 red
20-40 orange
40-50 yellow
50-80 blue-purpleish idk how to name that, you'll see :smile:
80-100 green
```
