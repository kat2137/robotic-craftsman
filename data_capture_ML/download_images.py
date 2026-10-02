import os
import sys
import subprocess
import urllib.parse

BASE_DIR = os.path.dirname(os.path.abspath(__file__)) #abspath makes it absolute path
OUT_DIR = os.path.join(BASE_DIR, "images")
PER_QUERY = int(os.environ.get("PER_QUERY", "200"))


def download(query: str) -> None:
    url = "https://www.pinterest.com/search/pins/?q=" + urllib.parse.quote(query)
    print(f"[INFO] downloading up to {PER_QUERY} images for '{query}' -> {OUT_DIR}")
    subprocess.run(
        [sys.executable, "-m", "gallery_dl", "--range", f"1-{PER_QUERY}", "-d", OUT_DIR, url],
        check=False,
    )


def main() -> None:
    queries = sys.argv[1:] or ["hand sewing"]
    for q in queries:
        download(q)
    n = len([f for f in os.listdir(OUT_DIR)]) if os.path.isdir(OUT_DIR) else 0
    print(f"[INFO] done — {n} files in {OUT_DIR}")


if __name__ == "__main__":
    main()
