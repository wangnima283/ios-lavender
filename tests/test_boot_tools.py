#!/usr/bin/env python3
"""Regression checks for real-ADT property filtering and boot-arg boundaries."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import adt
import mkadt
import mkpayload


class BootToolsTests(unittest.TestCase):
    def test_real_product_properties_ignore_bootloader_placeholders(self):
        product = adt.Node()
        product.props = [
            ('name', 0, b'product\0'),
            ('AAPL,phandle', 0, b'\x01\0\0\0'),
            ('artwork-device-subtype', 0, b'\x01\0\0\0'),
            ('artwork-display-gamut', 0, b'DCI-P3\0'),
            ('serial-number', 1, b'placeholder\0'),
        ]
        root = adt.Node()
        root.children = [product]
        with tempfile.TemporaryDirectory() as work:
            path = Path(work) / 'd22ap.adt'
            path.write_bytes(adt.serialize(root))
            self.assertEqual(mkadt.d22ap_product(path), [
                ('artwork-device-subtype', b'\x01\0\0\0'),
                ('artwork-display-gamut', b'DCI-P3\0'),
            ])

    def test_product_and_canvas_are_iphone_x_with_native_framebuffer(self):
        with patch.object(mkadt, 'd22ap_product', return_value=[('artwork-display-gamut', b'DCI-P3\0')]):
            root = mkadt.build(seed=bytes(64))
        children = {child.name: child for child in root.children}
        self.assertEqual(root.prop('model'), b'iPhone10,3\0')
        self.assertEqual(children['product'].prop('artwork-display-gamut'), b'sRGB\0')
        disp = next(n for n in children['arm-io'].children if n.name == 'disp0')
        self.assertEqual(disp.prop('canvas-width'), mkadt.u32(1125))
        self.assertEqual(disp.prop('canvas-height'), mkadt.u32(2436))
        self.assertEqual(children['sweet-fbcon'].prop('width'), mkadt.u32(1080))
        self.assertEqual(children['sweet-fbcon'].prop('height'), mkadt.u32(2340))

    def test_oversized_boot_arguments_fail_instead_of_losing_trailing_flags(self):
        with self.assertRaisesRegex(ValueError, '607 bytes'):
            mkpayload.build('unused', 'unused', 'x' * 608)
        with self.assertRaisesRegex(ValueError, '607 bytes'):
            mkpayload.build('unused', 'unused', '\u00e9' * 304)

    def test_embedded_nul_cannot_hide_boot_arguments(self):
        with self.assertRaisesRegex(ValueError, 'no NUL'):
            mkpayload.build('unused', 'unused', 'rd=md0\0rd=emmcroot0')


if __name__ == '__main__':
    unittest.main()
