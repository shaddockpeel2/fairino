#!/usr/bin/env python3
"""Download and verify the P5 TTS candidate assets.

Assets are written only inside this workspace.  The reference project is read
through (never written to), because it is mounted read-only and stays the
immutable provenance source for the Kokoro baseline.

Every downloaded file is recorded with its size and SHA256 in the candidate's
``manifest.json`` so later runs can verify offline with ``--verify-only``.
"""
import argparse
import bz2
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tarfile
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
REFERENCE = Path(os.environ.get('FR3C_QWEN_ASSET_ROOT', '/home/industio/DofbotPro_JetsonNano_0106'))
CONFIG = ROOT / 'config/voice/tts_candidates.json'
sys.dont_write_bytecode = True
USER_AGENT = 'fr3c-p5-tts-fetch/1'


def load_config():
    return json.loads(CONFIG.read_text())


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def download(url, destination, expected_bytes=None):
    """Download to a temporary sibling, then move it into place."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() and expected_bytes is not None and destination.stat().st_size == expected_bytes:
        print(f'    cached {destination.name} ({expected_bytes} bytes)')
        return destination
    request = urllib.request.Request(url, headers={'User-Agent': USER_AGENT})
    temporary = destination.with_suffix(destination.suffix + '.part')
    written = 0
    with urllib.request.urlopen(request, timeout=120) as response, temporary.open('wb') as handle:
        total = response.headers.get('Content-Length')
        total = int(total) if total and total.isdigit() else None
        while True:
            chunk = response.read(1024 * 1024)
            if not chunk:
                break
            handle.write(chunk)
            written += len(chunk)
            if total:
                print(f'\r    {destination.name}: {written * 100 // total}% ({written}/{total})', end='', flush=True)
    print()
    if expected_bytes is not None and written != expected_bytes:
        temporary.unlink(missing_ok=True)
        raise RuntimeError(f'{destination.name}: got {written} bytes, expected {expected_bytes}')
    temporary.replace(destination)
    return destination


def extract_archive(archive_path, destination, strip_root=True):
    """Extract a .tar.bz2 release asset, optionally dropping one top directory."""
    destination.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive_path, 'r:bz2') as handle:
        members = handle.getmembers()
        roots = {member.name.split('/')[0] for member in members if member.name}
        if strip_root and len(roots) == 1:
            root = roots.pop()
            for member in members:
                parts = member.name.split('/', 1)
                if len(parts) == 1:
                    continue
                member.name = parts[1]
                if member.issym() or member.islnk():
                    member.linkname = member.linkname.split('/', 1)[-1]
                target = destination / member.name
                if not str(target.resolve()).startswith(str(destination.resolve())):
                    raise RuntimeError(f'refusing path escape in archive: {member.name}')
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                elif member.isfile():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    source = handle.extractfile(member)
                    with target.open('wb') as out:
                        shutil.copyfileobj(source, out)
                elif member.issym():
                    continue
        else:
            handle.extractall(destination)
    return destination


def github_asset(url, filename, destination, expected_bytes=None):
    archive = destination / filename
    download(url, archive, expected_bytes)
    with tempfile.TemporaryDirectory(dir=destination) as staging:
        staging_path = Path(staging)
        extract_archive(archive, staging_path)
        # Release archives wrap their payload in a single top directory whose
        # name is not guaranteed to match the target directory name, so the
        # payload is flattened here instead of relying on names.
        entries = list(staging_path.iterdir())
        if len(entries) == 1 and entries[0].is_dir():
            source_root = entries[0]
        else:
            source_root = staging_path
        copied = 0
        for item in sorted(source_root.iterdir()):
            target = destination / item.name
            if item.is_dir():
                shutil.copytree(item, target, dirs_exist_ok=True)
            else:
                shutil.copy2(item, target)
            copied += 1
        if copied == 0:
            raise RuntimeError(f'{filename}: archive contained no payload')
    archive.unlink()


def huggingface_asset(base, repo, name, destination):
    url = f'{base}/{repo}/resolve/main/{name}'
    return download(url, destination / name)


def build(candidate_name, spec, config, verify_only):
    asset_root = ROOT / config['asset_root'] / candidate_name
    asset_root.mkdir(parents=True, exist_ok=True)
    print(f'  {candidate_name} -> {asset_root.relative_to(ROOT)}')

    if not verify_only and spec['source_kind'] == 'reference_project':
        source_dir = REFERENCE / spec['reference_dir']
        if not source_dir.is_dir():
            raise RuntimeError(f'reference assets missing: {source_dir}')
        for asset in spec['assets']:
            source = source_dir / asset['name']
            if not source.is_file():
                # Optional upstream extras are not required for the baseline.
                print(f'    skip (not in reference): {asset["name"]}')
                continue
            target = asset_root / asset['name']
            if not target.exists() or target.stat().st_size != source.stat().st_size:
                shutil.copy2(source, target)
            print(f'    copied {asset["name"]} ({source.stat().st_size} bytes)')
        espeak = spec.get('espeak_data_dir')
        if espeak and not (asset_root / espeak).is_dir():
            shutil.copytree(source_dir / espeak, asset_root / espeak,
                            ignore=shutil.ignore_patterns('*.pyc'))
            print(f'    copied {espeak}/')
        dict_dir = spec.get('dict_dir')
        if dict_dir and (source_dir / dict_dir).is_dir() and not (asset_root / dict_dir).is_dir():
            shutil.copytree(source_dir / dict_dir, asset_root / dict_dir)
            print(f'    copied {dict_dir}/')
    elif not verify_only and spec['source_kind'] == 'github_release':
        filename = spec['source_url'].rsplit('/', 1)[-1]
        github_asset(spec['source_url'], filename, asset_root)
        vocoder = spec.get('vocoder')
        if vocoder:
            download(vocoder['url'], asset_root / vocoder['name'], vocoder.get('bytes'))
    elif not verify_only and spec['source_kind'] == 'huggingface':
        for asset in spec['assets']:
            huggingface_asset(config['huggingface_base'], spec['repo'], asset['name'], asset_root)
    elif verify_only and spec['source_kind'] not in ('reference_project', 'github_release', 'huggingface'):
        raise RuntimeError(f'unknown source_kind: {spec["source_kind"]}')

    # In offline rebuild mode (verify_only=True) nothing is fetched, but the
    # manifest is still regenerated from whatever is on disk.
    files = []
    extras = [spec.get('vocoder')] if spec.get('vocoder') else []
    for asset in spec['assets'] + extras:
        path = asset_root / asset['name']
        if not path.is_file():
            # Optional assets (int8 variants) may legitimately be absent.
            continue
        files.append({'name': asset['name'], 'bytes': path.stat().st_size, 'sha256': sha256(path)})
    # Files the archive ships beyond the registry list (dictionaries, rule.far,
    # extra voices) are part of the asset and must be checksummed too.
    registered = {asset['name'] for asset in spec['assets'] + extras}
    for path in sorted(asset_root.rglob('*')):
        if not path.is_file() or path.name in registered or path.name == 'manifest.json':
            continue
        relative = str(path.relative_to(asset_root))
        files.append({'name': relative, 'bytes': path.stat().st_size, 'sha256': sha256(path)})
    manifest = {
        'schema_version': 1,
        'candidate': candidate_name,
        'role': spec.get('role'),
        'family': spec['family'],
        'license': spec.get('license'),
        'source_kind': spec['source_kind'],
        'source': spec.get('source_url') or spec.get('repo') or spec.get('reference_dir'),
        'fetched_utc': __import__('datetime').datetime.now(__import__('datetime').timezone.utc)
                              .strftime('%Y-%m-%dT%H:%M:%SZ'),
        'files': files,
        'total_bytes': sum(item['bytes'] for item in files),
        'sherpa_onnx': config['runtime']['sherpa_onnx'],
    }
    (asset_root / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(f'    manifest: {len(files)} files, {manifest["total_bytes"] / 1e6:.1f} MB')
    return manifest


def verify(candidate_name, spec, config):
    asset_root = ROOT / config['asset_root'] / candidate_name
    manifest_path = asset_root / 'manifest.json'
    if not manifest_path.is_file():
        return [f'{candidate_name}: manifest.json missing']
    manifest = json.loads(manifest_path.read_text())
    problems = []
    for record in manifest['files']:
        path = asset_root / record['name']
        if not path.is_file():
            problems.append(f'{candidate_name}/{record["name"]}: missing')
            continue
        if path.stat().st_size != record['bytes']:
            problems.append(f'{candidate_name}/{record["name"]}: size {path.stat().st_size} != {record["bytes"]}')
            continue
        actual = sha256(path)
        if actual != record['sha256']:
            problems.append(f'{candidate_name}/{record["name"]}: sha256 mismatch')
    return problems


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', action='append', default=None,
                        help='candidate key; repeatable. Default: all main candidates')
    parser.add_argument('--include-int8-control', action='store_true',
                        help='also fetch the quantised control-group assets')
    parser.add_argument('--verify-only', action='store_true',
                        help='offline check of existing assets against manifest.json')
    parser.add_argument('--manifest-only', action='store_true',
                        help='rebuild manifest.json from files already on disk, without downloading')
    args = parser.parse_args()

    config = load_config()
    candidates = config['candidates']
    if args.include_int8_control:
        for name, spec in config['int8_control_group']['candidates'].items():
            merged = dict(spec)
            if 'base_candidate' in spec:
                base = candidates[spec['base_candidate']]
                merged['assets'] = spec['assets']
                merged.setdefault('dict_dir', base.get('dict_dir'))
            candidates = {**candidates, name: merged}
    selected = args.candidate or [name for name in candidates if 'order' in candidates[name]]
    unknown = [name for name in selected if name not in candidates]
    if unknown:
        parser.error(f'unknown candidate(s): {unknown}')

    if args.verify_only:
        problems = []
        for name in selected:
            problems += verify(name, candidates[name], config)
        if problems:
            print('VERIFY FAILED')
            for problem in problems:
                print('  ' + problem)
            raise SystemExit(1)
        print(f'verified {len(selected)} candidate(s): all files match manifest.json')
        return

    if args.manifest_only:
        for name in selected:
            build(name, candidates[name], config, False)
        problems = []
        for name in selected:
            problems += verify(name, candidates[name], config)
        if problems:
            print('MANIFEST REBUILD VERIFY FAILED')
            for problem in problems:
                print('  ' + problem)
            raise SystemExit(1)
        print(f'rebuilt and verified {len(selected)} manifest(s) from files already on disk')
        return

    for name in selected:
        build(name, candidates[name], config, args.verify_only)

    problems = []
    for name in selected:
        problems += verify(name, candidates[name], config)
    if problems:
        print('POST-FETCH VERIFY FAILED')
        for problem in problems:
            print('  ' + problem)
        raise SystemExit(1)
    print(f'\nfetched and verified {len(selected)} candidate(s)')


if __name__ == '__main__':
    main()
