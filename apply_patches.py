from os.path import join, isfile

Import("env")

LIBDEPS_DIR = env.get("PROJECT_LIBDEPS_DIR")
ENV_NAME = env["PIOENV"]
SSLCLIENT_DIR = join(LIBDEPS_DIR, ENV_NAME, "SSLClient")
patchflag_path = join(SSLCLIENT_DIR, ".patching-done")

# patch file only if we didn't do it before
if not isfile(patchflag_path):
    original_file = join(SSLCLIENT_DIR, "src", "ssl_client.cpp")
    patched_file = join("patches", "esp32dev-sslclient-change-error-to-warning.patch")

    if not isfile(original_file):
        raise FileNotFoundError(f"Required file not found: {original_file}")
    if not isfile(patched_file):
        raise FileNotFoundError(f"Required file not found: {patched_file}")

    env.Execute("patch -u %s -i %s" % (original_file, patched_file))

    def _touch(path):
        with open(path, "w") as fp:
            fp.write("")

    env.Execute(lambda *args, **kwargs: _touch(patchflag_path))