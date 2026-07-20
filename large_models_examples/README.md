# large_models_examples

ROS 2 (ament_python) package reconstructed from the Hiwonder ROSOrin Pro tutorial
"[12. Large AI Model Courses](https://wiki.hiwonder.com/projects/rosorin-pro/en/latest/docs/12_Large_AI_Model_Courses.html)",
section **12.2 Multimodal Large Model Applications**. It wires a voice-activated
cloud LLM (OpenAI/OpenRouter via the robot's `large_models` package) to three
chassis behaviors:

| Node | Launch file | Voice command example |
|---|---|---|
| `llm_control_move` | `llm_control_move.launch.py` | "Move forward, then turn left" |
| `llm_visual_patrol` | `llm_visual_patrol.launch.py` | "Follow the black line and stop at obstacles" |
| `llm_color_track` | `llm_color_track.launch.py` | "Follow the red object" |

## Provenance

The tutorial page's "Program Analysis" subsections (12.2.3.5, 12.2.4.5, 12.2.5.5,
plus the offline variants in 12.5.3-12.5.5) show the actual `PROMPT` templates,
class state, `init_process`, `process()` main loops, and full launch files
verbatim - those were copied as-is. Small pieces the tutorial doesn't reprint
in full (import lists, `send_request`, `wakeup_callback`, `llm_result_callback`,
`get_node_state`, `main()`) are identical boilerplate shown for sibling nodes
in the same doc (e.g. `vllm_navigation_transport.py`, the offline
`llm_control_move_offline.py`), so they were filled in from those matching
snippets rather than invented from scratch.

## Runtime dependencies (not included here)

These nodes only make sense running on the actual ROSOrin Pro image, which
provides packages this tutorial doesn't ship source for:

- `controller` - chassis driver, exposes `/controller/cmd_vel` and `controller.launch.py`
- `app` - `line_following_node.launch.py`, `object_tracking_node.launch.py`
- `large_models` - `config.py` (`llm_model`, `api_key`, `base_url`, `start_audio_path`),
  the `speech` module, and `vocal_detect` / `agent_process` / `tts_node` launch files
- `large_models_msgs` - `SetModel`, `SetString`, `SetInt32` services
- `interfaces` - `SetColor` service (module path inferred from the sibling
  `SetPose2D`/`SetPoint`/`SetBox` services shown in the doc's navigation example;
  verify against the actual robot image)

Configure the API key in `~/large_models/config.py` per section 12.2.1 before
running, then:

```bash
sudo systemctl stop start_app_node.service
ros2 launch large_models_examples llm_control_move.launch.py
# or
ros2 launch large_models_examples llm_visual_patrol.launch.py
# or
ros2 launch large_models_examples llm_color_track.launch.py
```

Say "Hello Hiwonder" to wake the device, then speak a command.
