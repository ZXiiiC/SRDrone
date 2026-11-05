# Surviving notebook case

`notebook_detection_failure.xml` is the plan from `drone.md` lines 7603–7617. `notebook_detection_failure.txt` contains the actual recorded action lines 7622–7624. Only Markdown fences and surrounding blank lines were removed. The source notebook's hash is in `srdrone/prompts/manifest.json`.

Task: “Find and pass through a square frame.” The detector fails after takeoff and a yaw command. This case retains the historical plan, including its limited search/repeat structure; it is not a newly successful plan.

The independent synthetic integration demo is in `srdrone/data/frame_crossing/`. Its poses, scene, model responses and corrected plan are manually authored fixtures, not recovered experiment measurements.
