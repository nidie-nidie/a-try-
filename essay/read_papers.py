from pathlib import Path
import re

from pypdf import PdfReader


def compact(text: str) -> str:
    return re.sub(r"\s+", " ", text).strip()


for pdf in sorted(Path(__file__).parent.glob("*.pdf")):
    reader = PdfReader(str(pdf))
    text = compact(" ".join(page.extract_text() or "" for page in reader.pages))

    print(f"\n--- {pdf.name}")
    print(f"pages: {len(reader.pages)}")

    for marker in ("Abstract", "Conclusion", "Conclusions", "Discussion"):
        pos = text.lower().find(marker.lower())
        if pos != -1:
            print(compact(text[pos : pos + 1200]))
            break
