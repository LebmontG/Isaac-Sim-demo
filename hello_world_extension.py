import os
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.user_examples import HelloWorld
import asyncio
import omni.ui as ui
from omni.isaac.ui.ui_utils import btn_builder

class HelloWorldExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="gzh",
            submenu_name="",
            name="place",
            title="Hello World Example",
            doc_link="",
            overview="This is my test example",
            file_path=os.path.abspath(__file__),
            number_of_extra_frames=2,
            sample=HelloWorld(),
        )
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_task_controls_ui(frame)
        return

    def _on_stacking_button_event(self):
        asyncio.ensure_future(self.sample._on_stacking_event_async())
        self.task_ui_elements["Start"].enabled = False
        return

    def post_reset_button_event(self):
        self.task_ui_elements["Start"].enabled = True
        return

    def post_load_button_event(self):
        self.task_ui_elements["Start"].enabled = True
        return

    def post_clear_button_event(self):
        self.task_ui_elements["Start"].enabled = False
        return

    def build_task_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Task Controls"
                frame.visible = True
                dict = {
                    "label": "Start",
                    "type": "button",
                    "text": "Start",
                    "tooltip": "Start",
                    "on_clicked_fn": self._on_stacking_button_event,
                }

                self.task_ui_elements["Start"] = btn_builder(**dict)
                self.task_ui_elements["Start"].enabled = False

