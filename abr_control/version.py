name = "abr_control"
version_info = (0, 1, 0)  # (major, minor, patch)
dev = False

v = ".".join(str(v) for v in version_info)
dev_v = ".dev0" if dev else ""

version = f"{v}{dev_v}"
