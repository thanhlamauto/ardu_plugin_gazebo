# M7 result directory

Each run is written to `<scenario>/run_<NN>_seed_<seed>/` with a manifest,
structured JSONL events and launch log. Run
`python3 scripts/analyze_m7_results.py results/m7` after collecting data.

No validation result is committed here until the corresponding run has actually
been executed.
