import tkinter as tk
from tkinter import scrolledtext, messagebox, filedialog, Entry # 导入 Entry
import subprocess
import os
import threading

class ScriptRunnerApp:
    def __init__(self, master):
        self.master = master
        master.title("Mobile Robot Control Pad")
        master.geometry("750x600") # 调整窗口大小以容纳新元素

        # --- 脚本配置 ---
        self.scripts_to_run = [
            {"name": "Save Map", "path": "./catkin_workspace/src/launch_the_car/src/savemap.sh"},
            {"name": "Load Map", "path": "./catkin_workspace/src/launch_the_car/src/loadmap.sh"},
            {"name": "运行脚本 3 (错误)", "path": "./script3_error.sh"},
        ]
        self.file_processing_script_path = "./process_file.sh"
        self.text_input_script_path = "./catkin_workspace/src/launch_the_car/src/savemap.sh" # 新脚本的路径
        self.default_text_for_script = "My_Map" # 当输入框为空时使用的默认文本

        # --- GUI 元素 ---
        self.main_label = tk.Label(master, text="SLAM Mapping Mobile Robot Control Pad", font=("Arial", 18))
        self.main_label.pack(pady=10)

        # --- 常规脚本按钮 ---
        script_button_frame = tk.Frame(master)
        script_button_frame.pack(pady=5)
        self.buttons = [] # 用于统一管理所有按钮的状态
        for script_info in self.scripts_to_run:
            btn = tk.Button(script_button_frame, text=script_info["name"],
                            command=lambda s=script_info: self.run_script_thread(s["path"]))
            btn.pack(side=tk.LEFT, padx=5, pady=5)
            self.buttons.append(btn)

        # --- 文件处理按钮 ---
        file_process_frame = tk.Frame(master)
        file_process_frame.pack(pady=5)
        self.process_file_button = tk.Button(
            file_process_frame,
            text="LOAD MAP",
            command=self.handle_select_and_process_file,
            bg="lightblue"
        )
        self.process_file_button.pack(pady=5)
        self.buttons.append(self.process_file_button)

        # --- 带文本输入的脚本运行部分 ---
        text_input_frame = tk.Frame(master)
        text_input_frame.pack(pady=10)

        self.input_label = tk.Label(text_input_frame, text="Your map name:")
        self.input_label.pack(side=tk.LEFT, padx=(0, 5))

        self.text_entry = Entry(text_input_frame, width=40) # 创建文本输入框
        self.text_entry.pack(side=tk.LEFT, padx=5)
        self.text_entry.insert(0, "My_Map") # 设置占位符/初始文本

        self.run_with_text_button = tk.Button( # 创建新按钮
            text_input_frame,
            text="SAVE MAP",
            command=self.handle_run_with_input_text, # 按下时调用此方法
            bg="lightgreen" # 给按钮一个不同的背景色
        )
        self.run_with_text_button.pack(side=tk.LEFT, padx=5)
        self.buttons.append(self.run_with_text_button) # 将新按钮添加到 self.buttons 列表

        # --- 输出区域 ---
        self.output_label = tk.Label(master, text="脚本输出:", font=("Arial", 12))
        self.output_label.pack(pady=(10,0))
        self.output_text = scrolledtext.ScrolledText(master, height=20, width=90, wrap=tk.WORD)
        self.output_text.pack(pady=10, padx=10, fill=tk.BOTH, expand=True)
        self.output_text.configure(state='disabled')

    def _write_to_output(self, message):
        """帮助方法，用于向输出文本区域写入消息并滚动到底部。"""
        self.output_text.configure(state='normal') # 临时启用以允许插入
        self.output_text.insert(tk.END, message)
        self.output_text.see(tk.END) # 滚动到最后一行
        self.output_text.configure(state='disabled') # 再次禁用以变为只读
        self.master.update_idletasks() # 确保 GUI 更新

    def _set_buttons_state(self, state):
        """启用或禁用所有脚本按钮。"""
        for btn in self.buttons:
            btn.config(state=state)

    def handle_select_and_process_file(self):
        """打开文件对话框，然后使用所选文件运行 file_processing_script。"""
        file_path = filedialog.askopenfilename(
            title="选择一个文件进行处理",
            filetypes=(("所有文件", "*.*"),
                       ("Map File", "*.yaml")) # 示例文件类型
        )
        if file_path: # 用户选择了一个文件
            self._write_to_output(f"已选择文件: {file_path}\n")
            self._write_to_output(f"准备运行: {self.file_processing_script_path} 并传递参数: {file_path}\n")
            self.run_script_thread(self.file_processing_script_path, script_args=[file_path])
        else:
            self._write_to_output("文件选择已取消。\n")
            self._write_to_output("-----------------------------------------\n\n")

    def handle_run_with_input_text(self):
        """获取文本输入框的内容，并将其作为参数运行指定脚本。"""
        user_input = self.text_entry.get() # 获取输入框的当前文本

        argument_to_pass = "" # 初始化将要传递的参数
        pre_fix = './catkin_workspace/src/launch_the_car/map'
        fix = '.yaml'
        if user_input and user_input.strip(): # 检查输入是否非空且不仅仅是空格
            argument_to_pass = pre_fix + user_input + fix
            self._write_to_output(f"从输入框获取文本: '{argument_to_pass}'\n")
        else:
            # 如果输入为空或只有空白，则使用预设的默认字符串
            argument_to_pass = pre_fix + self.default_text_for_script + fix
            self._write_to_output(f"输入框为空，使用默认文本: '{argument_to_pass}'\n")

        # 记录将要执行的操作
        self._write_to_output(f"准备运行: {self.text_input_script_path} 并传递参数: '{argument_to_pass}'\n")
        # 调用现有的 run_script_thread 方法
        self.run_script_thread(self.text_input_script_path, script_args=[argument_to_pass])


    def run_script_thread(self, script_path, script_args=None): # 新增 script_args
        """在单独的线程中运行脚本执行，以防止 GUI 冻结。"""
        if script_args is None:
            script_args = []
        self._write_to_output(f"--- 尝试运行: {script_path} {' '.join(script_args)} ---\n")
        self._set_buttons_state(tk.DISABLED) # 脚本运行时禁用按钮

        thread = threading.Thread(target=self._execute_script_logic, args=(script_path, script_args))
        thread.daemon = True # 允许主程序即使线程正在运行也退出
        thread.start()

    def _execute_script_logic(self, script_path, script_args): # 新增 script_args
        """实际的脚本执行逻辑，在线程中运行。"""
        try:
            # 确保脚本路径是绝对的或相对于 Python 脚本目录正确
            if not os.path.isabs(script_path):
                # 假设脚本相对于 Python 脚本的目录
                base_dir = os.path.dirname(os.path.abspath(__file__))
                script_full_path = os.path.join(base_dir, script_path)
            else:
                script_full_path = script_path

            if not os.path.exists(script_full_path):
                self._write_to_output(f"错误: 脚本未在 {script_full_path} 找到\n")
                # self._write_to_output("-----------------------------------------\n\n") # 已移至 finally
                # self._set_buttons_state(tk.NORMAL) # 已移至 finally
                return # 提前退出

            # 构建命令
            base_command_parts = []
            if not os.access(script_full_path, os.X_OK):
                self._write_to_output(f"警告: 脚本 {script_full_path} 不可执行。尝试使用 'bash' 运行...\n")
                base_command_parts = ['bash', script_full_path]
            else:
                base_command_parts = [script_full_path] # 如果可执行，可以直接运行
            
            command_to_run = base_command_parts + script_args # 添加参数

            self._write_to_output(f"正在执行: {' '.join(command_to_run)}\n")

            # 运行脚本
            process = subprocess.Popen(command_to_run,
                                       stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE,
                                       text=True, # 将输出解码为字符串
                                       universal_newlines=True, # 更好地处理跨平台换行符
                                       cwd=os.path.dirname(script_full_path) # 在脚本所在目录运行
                                       )
            
            # 流式读取 stdout
            if process.stdout:
                for line in iter(process.stdout.readline, ''):
                    self._write_to_output(f"STDOUT: {line}")
                process.stdout.close()

            # 流式读取 stderr
            stderr_output = ""
            if process.stderr:
                for line in iter(process.stderr.readline, ''):
                    stderr_output += line
                    self._write_to_output(f"STDERR: {line}") # 实时显示错误
                process.stderr.close()
            
            return_code = process.wait() # 等待进程完成

            if return_code == 0:
                self._write_to_output(f"脚本 {os.path.basename(script_path)} 成功完成 (退出码: {return_code}).\n")
            else:
                self._write_to_output(f"脚本 {os.path.basename(script_path)} 执行失败 (退出码: {return_code}).\n")
                if not stderr_output and process.poll() is not None : # 如果 stderr 没有被完全读取
                     # 这个部分可能多余，如果 stderr 总是在上面被流式读取
                     remaining_stderr = process.stderr.read() if process.stderr else ""
                     if remaining_stderr: # 仅当有剩余错误时打印
                         self._write_to_output(f"附加 STDERR: {remaining_stderr}\n")


        except FileNotFoundError:
            self._write_to_output(f"错误: 脚本 '{script_path}' 或其解释器 (例如 bash) 未找到。\n")
        except PermissionError:
            self._write_to_output(f"错误: 执行 '{script_path}' 权限被拒绝。请确保它是可执行的 (`chmod +x {script_path}`).\n")
        except Exception as e:
            self._write_to_output(f"运行 {script_path} 时发生意外错误:\n{str(e)}\n")
        finally:
            self._write_to_output("-----------------------------------------\n\n")
            self._set_buttons_state(tk.NORMAL) # 重新启用按钮

def main():
    root = tk.Tk()
    app = ScriptRunnerApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()