"""The robot window's page script parses (2026-10-22: a "\\r" in the Python string became a
raw carriage return inside a JavaScript regex, so the whole script failed and Connect only
reloaded the form)."""
import os
import re
import shutil
import subprocess
import tempfile
import unittest

from can_driver import dclab_link


def page_script():
    match = re.search(r"<script>(.*)</script>", dclab_link.WINDOW_PAGE, re.S)
    return match.group(1)


class WindowPage(unittest.TestCase):
    def test_no_raw_carriage_return(self):
        self.assertNotIn("\r", dclab_link.WINDOW_PAGE)

    @unittest.skipUnless(shutil.which("node"), "node is not installed")
    def test_script_parses(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "window.js")
            with open(path, "w") as f:
                f.write(page_script())
            result = subprocess.run(["node", "--check", path], stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, universal_newlines=True)
        self.assertEqual(result.returncode, 0, result.stdout)


if __name__ == "__main__":
    unittest.main()
