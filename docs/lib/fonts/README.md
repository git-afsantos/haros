Minimizing font size
--------------------

To reduce the size of the output HAROS reports (to save space and increase loading speed)
it is possible to mimize the font to only those icons that are actually used.

This can be done with [fonttools](https://github.com/fonttools/fonttools) `pyftsubset` script (available via pip: `pip install fonttools`):

```
pyftsubset FontAwesome.otf --output-file=FontAwesome_min.otf --unicodes-file=codes.txt
```

where `codes.txt` is a text file containing the unicode code numbers of those characters that are being used and should therefore remain in the font.

The characters (icons) currently used in the HAROS report are:

```
fa-flag-o           f11d
fa-ellipsis-h       f141
fa-folder           f07b
fa-cogs             f085
fa-question         f128
fa-briefcase        f0b1
fa-line-chart       f201
fa-arrow-left       f060
fa-arrow-right      f061
fa-filter           f0b0
fa-hand-rock-o      f255
fa-font             f031
fa-server           f233
fa-search           f002
fa-info             f129
fa-spinner          f110
fa-file-code-o      f1c9
fa-bug              f188
fa-gears            f085
fa-exchange         f0ec
fa-close            f00d
fa-group            f0c0
fa-wrench           f0ad
fa-code-fork        f126
fa-warning          f071
fa-external-link    f08e
fa-sitemap          f0e8
fa-database         f1c0
fa-list-ol          f0cb
fa-terminal         f120
fa-file-text        f15c
fa-asterisk         f069
fa-question-circle  f059
```

So the `codes.txt` for this subset of characters should look like this:

```
U+0f11d
U+0f141
U+0f07b
U+0f085
U+0f128
U+0f0b1
U+0f201
U+0f060
U+0f061
U+0f0b0
U+0f255
U+0f031
U+0f233
U+0f002
U+0f129
U+0f110
U+0f1c9
U+0f188
U+0f085
U+0f0ec
U+0f00d
U+0f0c0
U+0f0ad
U+0f126
U+0f071
U+0f08e
U+0f0e8
U+0f1c0
U+0f0cb
U+0f120
U+0f15c
U+0f069
U+0f059
```
