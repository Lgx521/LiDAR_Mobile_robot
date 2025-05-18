import tkinter as tk
from tkinter import scrolledtext, messagebox, filedialog
import subprocess
import os
import threading # For running scripts in a separate thread to keep GUI responsive

class ScriptRunnerApp:
    def __init__(self, master):
        self.master = master
        master.title("Shell Script Runner")
        master.geometry("800x600") # Adjusted size for output area

        # --- Configuration for scripts ---
        # Each item is a dictionary: {"name": "Button Label", "path": "path/to/script.sh"}
        self.scripts_to_run = [
            {"name": "Save Map", "path": "./catkin_workspace/src/launch_the_car/src/savemap.sh"},
            {"name": "Load Map", "path": "./script2.sh"},
            {"name": "Run Script 3 (Error)", "path": "./script3_error.sh"},
            {"name": "Non-existent Script", "path": "./non_existent.sh"}
        ]

        # --- GUI Elements ---
        self.main_label = tk.Label(master, text="Welcome! Click a button to run a script.", font=("Arial", 14))
        self.main_label.pack(pady=10)

        # Frame for buttons
        button_frame = tk.Frame(master)
        button_frame.pack(pady=10)

        self.buttons = []
        for script_info in self.scripts_to_run:
            # Using lambda with a default argument to capture the current script_info
            btn = tk.Button(button_frame, text=script_info["name"], 
                            command=lambda s=script_info: self.run_script_thread(s["path"]))
            btn.pack(side=tk.LEFT, padx=5, pady=5)
            self.buttons.append(btn)

        # Output area
        self.output_label = tk.Label(master, text="Script Output:", font=("Arial", 12))
        self.output_label.pack(pady=(10,0))
        self.output_text = scrolledtext.ScrolledText(master, height=15, width=70, wrap=tk.WORD)
        self.output_text.pack(pady=10, padx=10, fill=tk.BOTH, expand=True)
        self.output_text.configure(state='disabled') # Make it read-only initially

    def _write_to_output(self, message):
        """Helper to write to the output text area and scroll to end."""
        self.output_text.configure(state='normal')
        self.output_text.insert(tk.END, message)
        self.output_text.see(tk.END) # Scroll to the end
        self.output_text.configure(state='disabled')
        self.master.update_idletasks() # Ensure GUI updates

    def _set_buttons_state(self, state):
        """Enable or disable all script buttons."""
        for btn in self.buttons:
            btn.config(state=state)

    def run_script_thread(self, script_path):
        """Runs the script execution in a separate thread to prevent GUI freezing."""
        self._write_to_output(f"--- Attempting to run: {script_path} ---\n")
        self._set_buttons_state(tk.DISABLED) # Disable buttons while script runs

        thread = threading.Thread(target=self._execute_script_logic, args=(script_path,))
        thread.daemon = True # Allows main program to exit even if thread is running
        thread.start()

    def _execute_script_logic(self, script_path):
        """The actual script execution logic, run in a thread."""
        try:
            # Ensure the script path is absolute or correctly relative
            if not os.path.isabs(script_path):
                # Assuming script is relative to the Python script's directory
                base_dir = os.path.dirname(os.path.abspath(__file__))
                script_full_path = os.path.join(base_dir, script_path)
            else:
                script_full_path = script_path

            if not os.path.exists(script_full_path):
                self._write_to_output(f"Error: Script not found at {script_full_path}\n")
                self._write_to_output("-----------------------------------------\n\n")
                self._set_buttons_state(tk.NORMAL)
                return

            if not os.access(script_full_path, os.X_OK):
                self._write_to_output(f"Warning: Script {script_full_path} is not executable. Attempting to run with 'bash'...\n")
                command = ['bash', script_full_path]
            else:
                command = [script_full_path] # If executable, can run directly

            # For .sh files, it's often safer to explicitly call with 'bash' or 'sh'
            # command = ['bash', script_full_path]

            # Run the script
            process = subprocess.Popen(command, 
                                       stdout=subprocess.PIPE, 
                                       stderr=subprocess.PIPE,
                                       text=True, # Decodes output to string
                                       universal_newlines=True) # For better cross-platform newline handling

            # Stream output
            self._write_to_output(f"Running: {' '.join(command)}\n")
            
            # Read stdout line by line
            if process.stdout:
                for line in iter(process.stdout.readline, ''):
                    self._write_to_output(f"STDOUT: {line}")
                process.stdout.close()

            # Read stderr line by line after stdout is exhausted (or interleave if preferred)
            stderr_output = ""
            if process.stderr:
                for line in iter(process.stderr.readline, ''):
                    stderr_output += line
                    self._write_to_output(f"STDERR: {line}") # Show errors as they come
                process.stderr.close()
            
            return_code = process.wait() # Wait for the process to complete

            if return_code == 0:
                self._write_to_output(f"Script {script_path} finished successfully (Exit Code: {return_code}).\n")
            else:
                self._write_to_output(f"Script {script_path} failed (Exit Code: {return_code}).\n")
                if not stderr_output: # If stderr wasn't streamed above for some reason
                     self._write_to_output(f"STDERR: {process.stderr.read() if process.stderr else 'N/A'}\n")


        except FileNotFoundError:
            self._write_to_output(f"Error: The script '{script_path}' or its interpreter (e.g., bash) was not found.\n")
        except PermissionError:
            self._write_to_output(f"Error: Permission denied to execute '{script_path}'. Make sure it's executable (`chmod +x {script_path}`).\n")
        except Exception as e:
            self._write_to_output(f"An unexpected error occurred while running {script_path}:\n{str(e)}\n")
        finally:
            self._write_to_output("-----------------------------------------\n\n")
            self._set_buttons_state(tk.NORMAL) # Re-enable buttons

def main():
    root = tk.Tk()
    app = ScriptRunnerApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()