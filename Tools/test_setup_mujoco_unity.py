"""Regression coverage for native visual pruning; no Unity/native runtime required."""

import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import setup_mujoco_unity as setup


class PhysicsOnlyModelTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        root = Path(self.temporary.name)
        self.source = root / "source"
        self.output = root / "generated"
        (self.source / "assets").mkdir(parents=True)
        self.source_xml = self.source / "robot_allcollisions.xml"
        self.source_xml.write_text("""<mujoco>
          <default><default class="microduck">
            <default class="visual"><geom type="mesh" contype="0" conaffinity="0"/></default>
            <default class="collision"><geom contype="1" conaffinity="1"/></default>
            <default class="self_collision_only"><geom contype="2" conaffinity="2"/></default>
          </default></default>
          <asset>
            <mesh name="contact" file="contact.stl"/>
            <mesh name="self" file="self.stl"/>
            <mesh name="visual" file="visual.stl"/>
            <material name="contact_material" rgba="1 0 0 1"/>
            <material name="visual_material" rgba="0 1 0 1"/>
          </asset>
          <worldbody><body name="trunk_base" childclass="microduck">
            <freejoint/>
            <inertial mass="1" pos="0 0 0" diaginertia="0.01 0.01 0.01"/>
            <geom name="decoration" class="visual" mesh="visual" material="visual_material"/>
            <geom name="shared_decoration" class="visual" mesh="contact"/>
            <geom name="foot" type="mesh" class="collision" mesh="contact" material="contact_material"/>
            <geom name="power" type="mesh" class="self_collision_only" mesh="self"/>
          </body></worldbody>
        </mujoco>""", encoding="utf-8")
        (self.source / "scene.xml").write_text(
            '<mujoco><include file="robot_allcollisions.xml"/><worldbody>'
            '<geom name="floor" type="plane" size="0 0 0.05"/>'
            '</worldbody></mujoco>', encoding="utf-8")
        for name in ("contact", "self", "visual"):
            self.write_mesh(name)
        paths = patch.multiple(setup, ROBOT_DIR=self.source, MODEL_OUT=self.output)
        paths.start()
        self.addCleanup(paths.stop)

    def write_mesh(self, name, scale=0.01):
        a = (0.0, 0.0, 0.0)
        b, c, d = (scale, 0.0, 0.0), (0.0, scale, 0.0), (0.0, 0.0, scale)
        triangles = ((a, c, b), (a, b, d), (a, d, c), (b, c, d))
        contents = b"MicroDuck asset-selection fixture".ljust(80, b"\0") + struct.pack("<I", 4)
        for triangle in triangles:
            contents += struct.pack("<12fH", 0, 0, 0, *(value for vertex in triangle for value in vertex), 0)
        (self.source / "assets" / (name + ".stl")).write_bytes(contents)

    def generated(self):
        return ET.parse(self.output / "scene.xml").getroot()

    def snapshot(self):
        return {path.relative_to(self.output).as_posix(): path.read_bytes()
                for path in self.output.rglob("*") if path.is_file()}

    def test_shared_collision_mesh_survives_visual_asset_removal(self):
        setup.make_model()
        model = self.generated()
        self.assertEqual({geom.get("name") for geom in model.findall("./worldbody//geom")}, {"foot", "power"})
        self.assertEqual({mesh.get("name") for mesh in model.findall("./asset/mesh")}, {"contact", "self"})
        self.assertEqual({material.get("name") for material in model.findall("./asset/material")}, {"contact_material"})
        self.assertEqual({path.name for path in (self.output / "assets").iterdir()},
                         {"contact.stl.bytes", "self.stl.bytes"})

    def test_collision_overrides_and_explicit_geom_dependencies_are_retained(self):
        source = ET.parse(self.source_xml)
        body = source.find("./worldbody/body")
        for name, overrides in (("sender", {"contype": "1"}), ("receiver", {"conaffinity": "1"}),
                                ("paired", {}), ("tracked", {})):
            ET.SubElement(body, "geom", name=name, type="mesh", mesh="visual", **{"class": "visual"}, **overrides)
        ET.SubElement(ET.SubElement(source.getroot(), "contact"), "pair", geom1="paired", geom2="foot")
        ET.SubElement(ET.SubElement(source.getroot(), "sensor"), "framepos", name="tracked_position",
                      objtype="geom", objname="tracked")
        source.write(self.source_xml)
        setup.make_model()
        model = self.generated()
        self.assertEqual({geom.get("name") for geom in model.findall("./worldbody//geom")},
                         {"foot", "power", "sender", "receiver", "paired", "tracked"})
        self.assertTrue((self.output / "assets/visual.stl.bytes").is_file())

    def test_regeneration_removes_obsolete_assets_without_losing_retained_guids(self):
        setup.make_model()
        assets = self.output / "assets"
        retained_meta = assets / "contact.stl.bytes.meta"
        retained_meta.write_text("fileFormatVersion: 2\nguid: 0123456789abcdef0123456789abcdef\n")
        (assets / "self.stl.bytes.meta").write_text("obsolete metadata")
        (assets / "orphan.stl.bytes.meta").write_text("orphan metadata")
        (assets / "notes.txt").write_text("not a generated native asset")
        source = ET.parse(self.source_xml)
        body = source.find("./worldbody/body")
        body.remove(body.find("geom[@name='power']"))
        source.write(self.source_xml)
        setup.make_model()
        self.assertEqual({path.name for path in assets.iterdir()},
                         {"contact.stl.bytes", "contact.stl.bytes.meta", "notes.txt"})
        self.assertIn("0123456789abcdef0123456789abcdef", retained_meta.read_text())
        before = self.snapshot()
        setup.make_model()
        self.assertEqual(self.snapshot(), before)

    def test_missing_new_collision_mesh_does_not_corrupt_previous_generation(self):
        setup.make_model()
        before = self.snapshot()
        self.write_mesh("contact", scale=0.02)
        source = ET.parse(self.source_xml)
        body = source.find("./worldbody/body")
        body.remove(body.find("geom[@name='power']"))
        ET.SubElement(body, "geom", name="new_contact", type="mesh", mesh="z_missing", **{"class": "collision"})
        ET.SubElement(source.find("asset"), "mesh", name="z_missing", file="z_missing.stl")
        source.write(self.source_xml)
        with self.assertRaisesRegex(RuntimeError, "Referenced mesh missing"):
            setup.make_model()
        self.assertEqual(self.snapshot(), before)


if __name__ == "__main__":
    unittest.main()
