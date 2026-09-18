"""
Compile the production zone parser with a recording stand-in, without hardware.

pick_place resolves its placement target from a zone string, so the grammar and
its rejections are part of the safety surface: a malformed zone must never be
silently treated as "use the default colour region".
"""

import shutil
import subprocess
from pathlib import Path

import pytest

HEADER = r'''
#include <cctype>
#include <cstddef>
#include <stdexcept>
#include <string>
'''

HARNESS = r'''
int main() {
  const auto accepted = [](const char * raw, const char * region, bool explicit_slot,
                           std::size_t slot_index) {
      ZoneRequest parsed;
      std::string detail;
      if (!parseZone(raw, parsed, detail)) {
        return false;
      }
      return parsed.region == region && parsed.explicit_slot == explicit_slot &&
             parsed.slot_index == slot_index;
    };
  const auto rejected = [](const char * raw) {
      ZoneRequest parsed;
      std::string detail;
      if (parseZone(raw, parsed, detail)) {
        return false;
      }
      return !detail.empty() && !parsed.explicit_slot && parsed.region.empty();
    };

  if (!accepted("A", "a", false, 0)) { return 1; }
  if (!accepted("a", "a", false, 0)) { return 2; }
  if (!accepted(" C ", "c", false, 0)) { return 3; }
  if (!accepted("A3", "a", true, 2)) { return 4; }
  if (!accepted("d1", "d", true, 0)) { return 5; }
  if (!accepted("B12", "b", true, 11)) { return 6; }
  if (!accepted("zone4", "zone", true, 3)) { return 7; }
  // Whitespace is ignored everywhere, including inside the zone string.
  if (!accepted("A 1", "a", true, 0)) { return 8; }

  if (!rejected("")) { return 20; }
  if (!rejected("   ")) { return 21; }
  if (!rejected("3")) { return 22; }
  if (!rejected("A0")) { return 23; }
  if (!rejected("A123")) { return 24; }
  if (!rejected("A-1")) { return 25; }
  if (!rejected("A1B")) { return 26; }
  if (!rejected("!")) { return 27; }
  return 0;
}
'''


def test_zone_grammar_accepts_regions_and_rejects_malformed_input(tmp_path):
    """Malformed zones must be refused, never silently defaulted."""
    if shutil.which('c++') is None:
        pytest.skip('no C++ compiler is available')
    source = (Path(__file__).parents[1] / 'src' / 'first_grasp_node.cpp').read_text()
    start = source.index('struct ZoneRequest')
    end = source.index('std::string stateName(int state)')
    unit = tmp_path / 'zone_parser.cpp'
    unit.write_text(HEADER + source[start:end] + HARNESS, encoding='utf-8')
    binary = tmp_path / 'zone_parser'
    compiled = subprocess.run(
        ['c++', '-std=c++17', '-Wall', '-Wextra', '-o', str(binary), str(unit)],
        capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr
    executed = subprocess.run([str(binary)], capture_output=True, text=True)
    assert executed.returncode == 0, (
        f'zone grammar case {executed.returncode} failed; stderr={executed.stderr}')


def test_pick_place_requires_a_zone_while_other_tasks_reject_one():
    """The four colour action servers stay mutually exclusive about `zone`."""
    source = (Path(__file__).parents[1] / 'src' / 'first_grasp_node.cpp').read_text()
    assert 'createPickServer("~/pick_hold", PickTaskMode::HOLD)' in source
    assert 'createPickServer("~/pick_sort", PickTaskMode::SORT)' in source
    assert 'createPickServer("~/pick_place", PickTaskMode::PLACE)' in source
    assert 'createPickServer("~/pick_stack", PickTaskMode::STACK)' in source
    guard = source.index('} else if (!raw_zone.empty()) {')
    assert 'use /fr3c_first_grasp/pick_place for an explicit placement zone' in \
        source[guard:guard + 400]
    # Only the place task may consume `zone`; every other task refuses it.
    assert 'if (mode == PickTaskMode::PLACE) {' in source[:guard]


def test_zone_slot_selection_reads_live_scene_instead_of_a_counter():
    """Zone occupancy must come from the planning scene so it self-heals."""
    source = (Path(__file__).parents[1] / 'src' / 'first_grasp_node.cpp').read_text()
    assert 'bool slotOccupied(const std::vector<double> & center)' in source
    occupied = source.index('bool slotOccupied(const std::vector<double> & center)')
    body = source[occupied:source.index('bool assignZoneSlot(', occupied)]
    assert 'planning_scene_.getObjects()' in body
    # The active target cube and the ground slab are never slot obstacles.
    assert 'entry.first == cube_id_ || entry.first == ground_plane_id_' in body
    # A zone task must never fall back to the colour-mapped region.
    assign = source.index('bool assignPlacementSlot(')
    assign_body = source[assign:source.index('bool lockTargetSnapshot(', assign)]
    assert 'if (!locked_zone_.region.empty()) {' in assign_body
    assert assign_body.index('assignZoneSlot(reason)') < \
        assign_body.index('place_centers_.at(color)')
