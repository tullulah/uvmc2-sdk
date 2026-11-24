#!/usr/bin/env python3
"""uvp.py — the Universal Vectrex Package: one file that installs a game on every board the
SDK builds for.

    uvp.py pack  --project DIR --name NAME [--um2 F] [--zero2 F] [--zero1 F] [--out F.uvp]
    uvp.py show  F.uvp

WHY A PACKAGE AND NOT A BINARY. The boards do not share a binary: the UVMC2 is an RP2350
(Cortex-M33, Thumb only, M profile), a PiTrex is a Raspberry Pi (Cortex-A, A profile), and the
two Pi Zeros differ again (ARMv8 with two cores to use, ARMv6 with one). So a package carries
one executable per board and each board's installer takes its own — a fat binary, the way the
old Universal Binaries were.

THE FILE is a zip, STORED (no compression): an installer then copies each file out as it
reads it, in pieces, with no inflater and no room for a whole file — which is what a board
with 8 MB of PSRAM and a 20 MB game needs. It holds:

    manifest.toml                 what this file describes (written by `pack`, below)
    uvmc2/<name>.um2              the UVMC2 cartridge's executable
    pitrex/zero2/<name>.img       the PiTrex's, Raspberry Pi Zero 2 W
    pitrex/zero1/<name>.img       the PiTrex's, the first Raspberry Pi Zero
    gamedata/...                  the files the game opens by relative path — its music, its
                                  settings as shipped. A board's installer puts them where its
                                  games look (on a PiTrex card gamedata/; on a UVMC2 card the
                                  top). Never ROMs of other people's games: a package that
                                  carries them cannot be given to anyone.

A board that did not build is simply absent; the manifest says which are there.

THE PROJECT'S SIDE is uvp.toml beside its Makefile (everything optional but the menu):

    [package]
    id          = "kuroishi"            # default: the build's name; unique on a card
    title       = "Kuroishi"            # default: the name, tidied
    version     = "1.1"
    author      = "Daniel Ferrer"
    description = "One line for the menu."
    menu        = "Originals"           # the menu group it goes in

    [needs]
    romsets = ["asteroid.zip"]          # looked for in roms/: the installer warns if missing

    [files]                             # more files: path in the package = path in the project
    "gamedata/kuroishi/KUROISHI.PAK" = "dist/KUROISHI.PAK"

    [[entry]]                           # more than one menu entry for the same executable
    title = "Web Wars"                  # (the Vectrex emulator: one per cartridge)
    param = "original/WEBWARS.BIN"

and a game whose build lays its card files out in build/card/ (a VPy game's music) gets them
as gamedata/ without saying so.

THE MANIFEST written into the package is that, plus what was found: [targets] (the path of
each board's executable in the package) and [[file]] (every other file, its size and CRC-32),
so an installer can check what it extracted and an uninstaller knows exactly what to remove.
"""
import argparse
import os
import sys
import tomllib
import zipfile
import zlib

FORMAT = 1
TARGETS = {          # manifest key -> folder in the package and the file's extension
    'uvmc2':        ('uvmc2', '.um2'),
    'pitrex_zero2': ('pitrex/zero2', '.img'),
    'pitrex_zero1': ('pitrex/zero1', '.img'),
}
DEFAULT_MENU = 'Games'


def toml_str(s):
    return '"' + str(s).replace('\\', '\\\\').replace('"', '\\"') + '"'


def tidy(name):
    return name.replace('_sdk', '').replace('aae_', '').replace('_', ' ').title()


def read_project(project):
    path = os.path.join(project, 'uvp.toml')
    if not os.path.exists(path):
        return {}
    with open(path, 'rb') as f:
        return tomllib.load(f)


def card_files(project):
    """build/card/<x> of a VPy game: the files it reads, laid out as on its card."""
    out = {}
    for root in (os.path.join(project, 'build', 'card'), os.path.join(project, '..', 'build', 'card')):
        if not os.path.isdir(root):
            continue
        for folder, _, files in os.walk(root):
            for name in files:
                src = os.path.join(folder, name)
                out['gamedata/' + os.path.relpath(src, root).replace(os.sep, '/')] = src
        break
    return out


def pack(a):
    spec = read_project(a.project)
    pkg = spec.get('package', {})
    ident = pkg.get('id', a.name)
    built = {'uvmc2': a.um2, 'pitrex_zero2': a.zero2, 'pitrex_zero1': a.zero1}
    targets = {}
    payload = {}
    for key, src in built.items():
        if src and os.path.isfile(src):
            folder, ext = TARGETS[key]
            inner = '%s/%s%s' % (folder, a.name, ext)
            targets[key] = inner
            payload[inner] = src
    if not targets:
        sys.exit('uvp: nothing built for any board')
    files = card_files(a.project)
    for inner, src in spec.get('files', {}).items():
        files[inner] = os.path.join(a.project, src)
    for inner, src in files.items():
        if not inner.startswith('gamedata/'):
            sys.exit('uvp: %s: a package\'s files go under gamedata/' % inner)
        if not os.path.isfile(src):
            sys.exit('uvp: %s: %s is not there' % (inner, src))
        payload[inner] = src

    lines = ['# Universal Vectrex Package — written by uvmc2-sdk/tools/uvp.py', 'format = %d' % FORMAT, '',
             '[package]']
    lines += ['id = %s' % toml_str(ident),
              'title = %s' % toml_str(pkg.get('title', tidy(a.name))),
              'version = %s' % toml_str(pkg.get('version', '0')),
              'author = %s' % toml_str(pkg.get('author', '')),
              'description = %s' % toml_str(pkg.get('description', '')),
              'menu = %s' % toml_str(pkg.get('menu', DEFAULT_MENU)),
              'name = %s' % toml_str(a.name), '', '[needs]',
              'romsets = [%s]' % ', '.join(toml_str(r) for r in spec.get('needs', {}).get('romsets', [])),
              '', '[targets]']
    lines += ['%s = %s' % (k, toml_str(v)) for k, v in targets.items()]
    for e in spec.get('entry', []):
        lines += ['', '[[entry]]', 'title = %s' % toml_str(e['title']), 'param = %s' % toml_str(e.get('param', ''))]
        if 'menu' in e:
            lines.append('menu = %s' % toml_str(e['menu']))
    for inner in sorted(p for p in payload if p.startswith('gamedata/')):
        data = open(payload[inner], 'rb').read()
        lines += ['', '[[file]]', 'path = %s' % toml_str(inner), 'size = %d' % len(data),
                  'crc32 = %d' % (zlib.crc32(data) & 0xffffffff)]
    manifest = '\n'.join(lines) + '\n'

    out = a.out or os.path.join(a.project, 'build_uvp', ident + '.uvp')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    tmp = out + '.tmp'
    with zipfile.ZipFile(tmp, 'w', zipfile.ZIP_STORED) as z:
        z.writestr('manifest.toml', manifest)
        for inner in sorted(payload):
            z.write(payload[inner], inner)
    os.replace(tmp, out)
    print('uvp: %s (%d bytes): %s; %d data files' % (out, os.path.getsize(out), ', '.join(targets),
                                                      len(payload) - len(targets)))


def show(a):
    with zipfile.ZipFile(a.file) as z:
        m = tomllib.loads(z.read('manifest.toml').decode())
        print('%s %s (%s), menu "%s"' % (m['package']['title'], m['package']['version'], m['package']['id'],
                                         m['package']['menu']))
        for k, v in m.get('targets', {}).items():
            print('  %-13s %s (%d bytes)' % (k, v, z.getinfo(v).file_size))
        print('  %d data files, needs %s' % (len(m.get('file', [])), m['needs']['romsets'] or 'nothing'))
        bad = [f['path'] for f in m.get('file', []) if zlib.crc32(z.read(f['path'])) & 0xffffffff != f['crc32']]
        print('  contents ' + ('OK' if not bad else 'DAMAGED: ' + ', '.join(bad)))


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    k = sub.add_parser('pack')
    k.add_argument('--project', required=True)
    k.add_argument('--name', required=True)
    k.add_argument('--um2')
    k.add_argument('--zero2')
    k.add_argument('--zero1')
    k.add_argument('--out')
    s = sub.add_parser('show')
    s.add_argument('file')
    a = p.parse_args()
    {'pack': pack, 'show': show}[a.cmd](a)


if __name__ == '__main__':
    main()
