#!/usr/bin/env python3
import sys
import pandas as pd

if len(sys.argv) != 3:
    print("Usage: convert_contacts_only.py input.csv output.csv")
    sys.exit(1)

inp, out = sys.argv[1], sys.argv[2]

df = pd.read_csv(inp)

contact_cols = [c for c in df.columns if c.startswith("contact_")]

if not contact_cols:
    raise RuntimeError("No contact_* columns found")

for c in contact_cols:
    df[c] = df[c].map({
        True: 1, False: 0,
        "True": 1, "False": 0,
        "true": 1, "false": 0
    })

# Check if any NaN remain (means unexpected values)
if df[contact_cols].isna().any().any():
    bad = df[contact_cols][df[contact_cols].isna().any(axis=1)].head()
    print("ERROR: Some contact values could not be converted:")
    print(bad)
    sys.exit(1)

df.to_csv(out, index=False)
print(f"Wrote {out}")
