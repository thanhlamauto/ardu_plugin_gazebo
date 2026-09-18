# M7 result directory

Each run is written to `<scenario>/run_<NN>_seed_<seed>/` with a manifest,
structured JSONL events and launch log. Run
`python3 scripts/analyze_m7_results.py results/m7` after collecting data.

Campaign M7.1 đã chạy ngày 2026-09-18. Aggregate CSV, manifest, report và đồ thị
được lưu tại [`../m7_campaign_7e47663_20260918/`](../m7_campaign_7e47663_20260918/).
Raw JSONL/log 90 MB được giữ ngoài Git; không dùng thư mục aggregate thay cho
raw source khi cần phân tích lại.
