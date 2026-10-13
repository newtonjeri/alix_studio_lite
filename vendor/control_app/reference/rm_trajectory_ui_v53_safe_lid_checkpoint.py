#!/usr/bin/env python3
"""
RealMan Trajectory UI v53

Simple GUI wrapper around rm_upload.py.

Features:
- Select a trajectory .txt file
- Choose controller program slot 1-100
- Upload only
- Multi-file batch upload with per-file Program IDs and replacement confirmation
- Batch file browser supports individual selection, Select All, and Select Folder Files
- Upload + verify + run
- Run the current upload-slot program without uploading again
- Run an existing controller program by ID or program name
- Multi-run stored programs with editable Hand/Pole/Wait table and one confirmation
- Manual Hand and Pole sliders with independent Apply buttons
- Synchronized Dual Run: one stored program on LEFT and RIGHT starts concurrently
- Drag & Teach: record, local save, and playback per arm
- Save Multi Run sequences locally and load/edit/run them later
- Refresh controller program list and choose from a dropdown
- One GUI confirmation before any action that can move the robot
- Live terminal output and final status
- High-visibility white/cyan text cursor in editable fields, including Upload slot
- Slow Stop button sends Ctrl+C/SIGINT to rm_upload.py, which performs
  rm_set_arm_slow_stop() during an active run

Keep this file in the same folder as rm_upload.py, or select rm_upload.py
using the "Browse" button.
"""

import os
import json
import re
import signal
import subprocess
import sys
import threading
import tempfile
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk


UI_PROGRAM_VERSION = "v53"
EXPECTED_RUNNER_VERSION = "v14"

DEFAULT_SDK = os.environ.get(
    "ALIX_RM_SDK",
    "/home/alai-labs/alix_ws/src/alix/external/butterfli-realman-alix/Python",
)


# ---------------------------------------------------------------------------
# Robotics technology UI palette
# ---------------------------------------------------------------------------
BG = "#06111F"
PANEL = "#0B1B2B"
PANEL_2 = "#10263C"
ENTRY_BG = "#071827"
BORDER = "#1B4965"

TEXT = "#EAF7FF"
MUTED = "#86A6BA"

CYAN = "#00E5FF"
BLUE = "#3B82F6"
PURPLE = "#8B5CF6"
GREEN = "#00D084"
ORANGE = "#FF9F1C"
RED = "#FF3B5C"


class TrajectoryFileDialog(tk.Toplevel):
    """Simple searchable .txt file browser for trajectory files."""

    def __init__(self, parent, initialdir=None, multiple=False):
        super().__init__(parent)
        self.multiple = bool(multiple)
        self.title("Select trajectory files" if self.multiple else "Select trajectory file")
        self.geometry("980x650")
        self.minsize(760, 500)
        self.configure(bg=BG)
        self.transient(parent)

        self.result = None
        self.history = []
        self.history_index = -1

        home = Path.home()
        downloads = home / "Downloads"
        requested = Path(initialdir).expanduser() if initialdir else downloads

        if requested.is_dir():
            start_dir = requested
        elif downloads.is_dir():
            start_dir = downloads
        else:
            start_dir = home

        self.current_dir = start_dir.resolve()
        self.search_var = tk.StringVar()
        self.path_var = tk.StringVar(value=str(self.current_dir))
        self.selected_var = tk.StringVar()
        self.info_var = tk.StringVar()

        # File-browser sorting.
        # Default: latest modified item first.
        self.sort_column = "modified"
        self.sort_descending = True

        self._build()
        self._push_history(self.current_dir)
        self._refresh()

        self.search_var.trace_add("write", lambda *_: self._refresh())

        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self.bind("<Escape>", lambda _e: self._cancel())
        self.bind("<Alt-Left>", lambda _e: self._back())
        self.bind("<BackSpace>", self._backspace_if_not_typing)

        self.grab_set()
        self.search_entry.focus_set()

    def _build(self):
        outer = ttk.Frame(self, padding=12, style="App.TFrame")
        outer.pack(fill="both", expand=True)

        # Navigation toolbar
        toolbar = ttk.Frame(outer)
        toolbar.pack(fill="x")

        self.back_btn = ttk.Button(toolbar, text="← Back", command=self._back)
        self.back_btn.pack(side="left", padx=(0, 5))

        ttk.Button(toolbar, text="↑ Up", command=self._up).pack(side="left", padx=5)
        ttk.Button(toolbar, text="⌂ Home", command=self._home).pack(side="left", padx=5)
        ttk.Button(toolbar, text="Downloads", command=self._downloads).pack(side="left", padx=5)
        ttk.Button(toolbar, text="Refresh", command=self._refresh).pack(side="left", padx=5)

        # Path bar
        path_frame = ttk.Frame(outer)
        path_frame.pack(fill="x", pady=(10, 6))

        ttk.Label(path_frame, text="Folder:").pack(side="left", padx=(0, 6))
        path_entry = tk.Entry(
            path_frame,
            textvariable=self.path_var,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 10),
        )
        path_entry.pack(side="left", fill="x", expand=True, ipady=5)
        path_entry.bind("<Return>", lambda _e: self._go_path())
        ttk.Button(path_frame, text="Go", command=self._go_path).pack(side="left", padx=(6, 0))

        # Search box
        search_frame = ttk.Frame(outer)
        search_frame.pack(fill="x", pady=(2, 8))

        ttk.Label(search_frame, text="Search files:").pack(side="left", padx=(0, 6))
        self.search_entry = tk.Entry(
            search_frame,
            textvariable=self.search_var,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 10),
        )
        self.search_entry.pack(side="left", fill="x", expand=True, ipady=5)
        self.search_entry.bind("<Return>", lambda _e: self._open_first_match())
        ttk.Button(
            search_frame,
            text="Clear",
            command=lambda: self.search_var.set("")
        ).pack(side="left", padx=(6, 0))

        # Batch-selection toolbar. These controls are only useful when the
        # dialog is being used by MULTI UPLOAD.
        if self.multiple:
            batch_tools = ttk.Frame(outer, style="App.TFrame")
            batch_tools.pack(fill="x", pady=(0, 8))

            ttk.Button(
                batch_tools,
                text="✓  SELECT ALL FILES",
                command=self._select_all_visible_files,
                style="List.TButton",
            ).pack(side="left", padx=(0, 8))

            ttk.Button(
                batch_tools,
                text="📁  SELECT FOLDER FILES",
                command=self._select_folder_files,
                style="Upload.TButton",
            ).pack(side="left", padx=8)

            ttk.Label(
                batch_tools,
                text="Select individual files, all visible files, or all .txt files from a folder.",
                style="Muted.TLabel",
            ).pack(side="left", padx=(10, 0))

        # File list
        tree_frame = ttk.Frame(outer)
        tree_frame.pack(fill="both", expand=True)

        self.tree = ttk.Treeview(
            tree_frame,
            columns=("type", "size", "date", "time"),
            show="tree headings",
            selectmode="extended" if self.multiple else "browse",
        )

        # Clicking a heading sorts the visible items.
        self.tree.heading(
            "#0", text="Name", anchor="w",
            command=lambda: self._set_sort("name")
        )
        self.tree.heading(
            "type", text="Type", anchor="w",
            command=lambda: self._set_sort("type")
        )
        self.tree.heading(
            "size", text="Size", anchor="e",
            command=lambda: self._set_sort("size")
        )
        self.tree.heading(
            "date", text="Date Modified ▼", anchor="center",
            command=lambda: self._set_sort("modified")
        )
        self.tree.heading(
            "time", text="Time Modified ▼", anchor="center",
            command=lambda: self._set_sort("modified")
        )

        self.tree.column("#0", width=430, minwidth=240, stretch=True)
        self.tree.column("type", width=90, minwidth=75, stretch=False)
        self.tree.column("size", width=95, minwidth=75, anchor="e", stretch=False)
        self.tree.column("date", width=115, minwidth=100, anchor="center", stretch=False)
        self.tree.column("time", width=105, minwidth=90, anchor="center", stretch=False)

        ybar = ttk.Scrollbar(tree_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=ybar.set)

        self.tree.pack(side="left", fill="both", expand=True)
        ybar.pack(side="right", fill="y")

        self.tree.bind("<Double-1>", self._double_click)
        self.tree.bind("<<TreeviewSelect>>", self._selection_changed)
        self.tree.bind("<Return>", lambda _e: self._activate_selected())

        # Selected filename / status
        bottom = ttk.Frame(outer)
        bottom.pack(fill="x", pady=(10, 0))

        ttk.Label(
            bottom,
            text="Selected:" if self.multiple else "File name:",
        ).grid(row=0, column=0, sticky="w", padx=(0, 6))
        selected_entry = tk.Entry(
            bottom,
            textvariable=self.selected_var,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 10),
        )
        selected_entry.grid(row=0, column=1, sticky="ew", ipady=5)
        selected_entry.bind("<Return>", lambda _e: self._open_selected())

        ttk.Label(bottom, textvariable=self.info_var).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(7, 0)
        )

        buttons = ttk.Frame(bottom)
        buttons.grid(row=0, column=2, rowspan=2, sticky="e", padx=(12, 0))

        self.open_btn = ttk.Button(
            buttons,
            text="Select Files" if self.multiple else "Open",
            command=self._open_selected,
        )
        self.open_btn.pack(side="left", padx=(0, 6))
        ttk.Button(buttons, text="Cancel", command=self._cancel).pack(side="left")

        bottom.columnconfigure(1, weight=1)

    @staticmethod
    def _format_size(size):
        if size < 1024:
            return f"{size} B"
        if size < 1024 * 1024:
            return f"{size / 1024:.1f} KB"
        return f"{size / (1024 * 1024):.1f} MB"

    @staticmethod
    def _file_info(path):
        """Return size, modified timestamp, display date and display time."""
        try:
            stat = path.stat()
            size = stat.st_size if path.is_file() else 0
            modified = stat.st_mtime
            dt = __import__("datetime").datetime.fromtimestamp(modified)
            date_text = dt.strftime("%Y-%m-%d")
            time_text = dt.strftime("%H:%M:%S")
            return size, modified, date_text, time_text
        except OSError:
            return 0, 0.0, "", ""

    def _set_sort(self, column):
        """Sort a column. Date/time starts newest-first; others start ascending."""
        if self.sort_column == column:
            self.sort_descending = not self.sort_descending
        else:
            self.sort_column = column
            self.sort_descending = (column == "modified")

        self._refresh()

    def _update_sort_headings(self):
        # Remove old arrows first.
        labels = {
            "#0": "Name",
            "type": "Type",
            "size": "Size",
            "date": "Date Modified",
            "time": "Time Modified",
        }

        arrow = "▼" if self.sort_descending else "▲"

        if self.sort_column == "name":
            labels["#0"] += f" {arrow}"
        elif self.sort_column == "type":
            labels["type"] += f" {arrow}"
        elif self.sort_column == "size":
            labels["size"] += f" {arrow}"
        elif self.sort_column == "modified":
            labels["date"] += f" {arrow}"
            labels["time"] += f" {arrow}"

        self.tree.heading("#0", text=labels["#0"])
        self.tree.heading("type", text=labels["type"])
        self.tree.heading("size", text=labels["size"])
        self.tree.heading("date", text=labels["date"])
        self.tree.heading("time", text=labels["time"])

    def _refresh(self):
        if not self.current_dir.is_dir():
            return

        for item in self.tree.get_children():
            self.tree.delete(item)

        query = self.search_var.get().strip().lower()

        try:
            entries = list(self.current_dir.iterdir())
        except PermissionError:
            messagebox.showerror(
                "Permission denied",
                f"Cannot open:\n{self.current_dir}",
                parent=self,
            )
            return
        except OSError as exc:
            messagebox.showerror("Folder error", str(exc), parent=self)
            return

        # Directories stay visible so the user can navigate into other folders.
        dirs = [p for p in entries if p.is_dir()]

        # Trajectory browser shows .txt files only.
        files = [
            p for p in entries
            if p.is_file()
            and p.suffix.lower() == ".txt"
            and (not query or query in p.name.lower())
        ]

        # Build one sortable list. Date/time sorting uses the same modified timestamp,
        # so "latest" means the newest modified item.
        rows = []
        for p in dirs + files:
            size_bytes, modified, date_text, time_text = self._file_info(p)
            is_dir = p.is_dir()
            rows.append({
                "path": p,
                "is_dir": is_dir,
                "name": p.name.lower(),
                "type": "folder" if is_dir else "txt file",
                "size": size_bytes,
                "modified": modified,
                "date": date_text,
                "time": time_text,
            })

        if self.sort_column == "name":
            keyfunc = lambda r: r["name"]
        elif self.sort_column == "type":
            keyfunc = lambda r: (r["type"], r["name"])
        elif self.sort_column == "size":
            keyfunc = lambda r: (r["size"], r["name"])
        else:
            keyfunc = lambda r: (r["modified"], r["name"])

        rows.sort(key=keyfunc, reverse=self.sort_descending)

        for row in rows:
            p = row["path"]
            if row["is_dir"]:
                display_name = "📁 " + p.name
                type_text = "Folder"
                size_text = ""
                tag = ("dir",)
            else:
                display_name = p.name
                type_text = "TXT file"
                size_text = self._format_size(row["size"])
                tag = ("file",)

            self.tree.insert(
                "",
                "end",
                iid=str(p),
                text=display_name,
                values=(
                    type_text,
                    size_text,
                    row["date"],
                    row["time"],
                ),
                tags=tag,
            )

        self._update_sort_headings()

        self.path_var.set(str(self.current_dir))
        self.selected_var.set("")
        self._update_back_button()

        if query:
            self.info_var.set(
                f'{len(files)} matching .txt file(s) in {self.current_dir.name or self.current_dir}'
            )
        else:
            self.info_var.set(
                f"{len(files)} .txt file(s), {len(dirs)} folder(s)"
            )

        if self.multiple and query:
            self.info_var.set(
                self.info_var.get() + "  •  Select All uses only these visible matches"
            )

    def _push_history(self, path):
        path = Path(path).resolve()
        if self.history_index >= 0 and self.history[self.history_index] == path:
            return

        # Remove forward history if the user navigated back and then elsewhere.
        self.history = self.history[: self.history_index + 1]
        self.history.append(path)
        self.history_index = len(self.history) - 1
        self._update_back_button()

    def _navigate(self, path, add_history=True):
        path = Path(path).expanduser()
        try:
            path = path.resolve()
        except OSError:
            pass

        if not path.is_dir():
            messagebox.showerror(
                "Folder not found",
                f"This folder does not exist:\n{path}",
                parent=self,
            )
            return

        self.current_dir = path
        self.search_var.set("")
        if add_history:
            self._push_history(path)
        self._refresh()

    def _back(self):
        if self.history_index > 0:
            self.history_index -= 1
            self.current_dir = self.history[self.history_index]
            self.search_var.set("")
            self._refresh()

    def _up(self):
        parent = self.current_dir.parent
        if parent != self.current_dir:
            self._navigate(parent)

    def _home(self):
        self._navigate(Path.home())

    def _downloads(self):
        downloads = Path.home() / "Downloads"
        if downloads.is_dir():
            self._navigate(downloads)
        else:
            messagebox.showinfo(
                "Downloads",
                f"Downloads folder not found:\n{downloads}",
                parent=self,
            )

    def _go_path(self):
        typed = self.path_var.get().strip()
        if typed:
            self._navigate(Path(typed))

    def _update_back_button(self):
        self.back_btn.configure(
            state="normal" if self.history_index > 0 else "disabled"
        )

    def _visible_txt_items(self):
        """Return Treeview item IDs for all currently visible .txt files."""
        items = []
        for item in self.tree.get_children():
            try:
                p = Path(item)
            except Exception:
                continue
            if p.is_file() and p.suffix.lower() == ".txt":
                items.append(item)
        return items

    def _select_all_visible_files(self):
        """Select every visible .txt file, respecting the active search filter."""
        if not self.multiple:
            return

        items = self._visible_txt_items()
        if not items:
            messagebox.showinfo(
                "Select All Files",
                "No visible .txt trajectory files are available in this folder.",
                parent=self,
            )
            return

        self.tree.selection_set(items)
        self.tree.focus(items[0])
        self.tree.see(items[0])
        self.selected_var.set(f"{len(items)} trajectory files selected")

    def _select_folder_files(self):
        """
        Choose a folder and return every .txt file directly inside that folder.

        This is intentionally non-recursive: files in subfolders are not
        included automatically.
        """
        if not self.multiple:
            return

        initial = str(self.current_dir) if self.current_dir.is_dir() else str(Path.home())

        folder = filedialog.askdirectory(
            title="Select folder containing trajectory files",
            initialdir=initial,
            parent=self,
        )
        if not folder:
            return

        folder_path = Path(folder)
        try:
            files = [
                p for p in folder_path.iterdir()
                if p.is_file() and p.suffix.lower() == ".txt"
            ]
        except (PermissionError, OSError) as exc:
            messagebox.showerror(
                "Folder error",
                f"Could not read this folder:\\n{folder_path}\\n\\n{exc}",
                parent=self,
            )
            return

        if not files:
            messagebox.showinfo(
                "No trajectory files",
                f"No .txt files were found directly inside:\\n{folder_path}",
                parent=self,
            )
            return

        # Latest modified file first, matching the normal browser default.
        def modified_time(path):
            try:
                return path.stat().st_mtime
            except OSError:
                return 0

        files.sort(key=modified_time, reverse=True)

        ok = messagebox.askyesno(
            "Select Folder Files",
            f"Folder:\\n{folder_path}\\n\\n"
            f"Found {len(files)} .txt file(s).\\n\\n"
            "Add all of these files to the batch upload?",
            parent=self,
        )
        if not ok:
            return

        self.result = [str(p) for p in files]
        self.destroy()

    def _selection_changed(self, _event=None):
        selection = self.tree.selection()
        if not selection:
            self.selected_var.set("")
            return

        file_paths = [Path(item) for item in selection if Path(item).is_file()]

        if self.multiple:
            if not file_paths:
                self.selected_var.set("")
            elif len(file_paths) == 1:
                self.selected_var.set(file_paths[0].name)
            else:
                self.selected_var.set(f"{len(file_paths)} trajectory files selected")
            return

        path = Path(selection[0])
        if path.is_file():
            self.selected_var.set(path.name)
        else:
            self.selected_var.set("")

    def _double_click(self, _event=None):
        self._activate_selected()

    def _activate_selected(self):
        selection = self.tree.selection()
        if not selection:
            return

        # If exactly one selected item is a folder, navigate into it.
        if len(selection) == 1:
            path = Path(selection[0])
            if path.is_dir():
                self._navigate(path)
                return

        file_paths = [Path(item) for item in selection if Path(item).is_file()]
        if not file_paths:
            return

        if self.multiple:
            self.result = [str(p) for p in file_paths]
        else:
            self.result = str(file_paths[0])
        self.destroy()

    def _open_selected(self):
        selection = self.tree.selection()

        if selection:
            # Single selected folder means navigation, not selection.
            if len(selection) == 1:
                only = Path(selection[0])
                if only.is_dir():
                    self._navigate(only)
                    return

            file_paths = [Path(item) for item in selection if Path(item).is_file()]
            if file_paths:
                if self.multiple:
                    self.result = [str(p) for p in file_paths]
                else:
                    self.result = str(file_paths[0])
                self.destroy()
                return

        # Manual filename entry is only used for single-file mode.
        if not self.multiple:
            typed = self.selected_var.get().strip()
            if typed:
                candidate = self.current_dir / typed
                if candidate.is_file() and candidate.suffix.lower() == ".txt":
                    self.result = str(candidate)
                    self.destroy()
                    return

        messagebox.showinfo(
            "Select trajectory",
            "Select one or more trajectory .txt files first."
            if self.multiple
            else "Select a trajectory .txt file first.",
            parent=self,
        )

    def _open_first_match(self):
        files = []
        for item in self.tree.get_children():
            p = Path(item)
            if p.is_file():
                files.append(item)

        if len(files) == 1:
            self.tree.selection_set(files[0])
            self.tree.focus(files[0])
            self._activate_selected()
        elif files:
            self.tree.selection_set(files[0])
            self.tree.focus(files[0])
            self.tree.see(files[0])

    def _backspace_if_not_typing(self, event):
        widget = self.focus_get()
        if isinstance(widget, (tk.Entry, ttk.Entry)):
            return
        self._up()

    def _cancel(self):
        self.result = None
        self.destroy()

    def show(self):
        self.wait_window()
        return self.result


class BatchUploadDialog(tk.Toplevel):
    """Assign one controller Program ID to each selected trajectory file."""

    def __init__(self, parent, files, start_slot=1):
        super().__init__(parent)
        self.title("Batch Trajectory Upload Setup")
        self.geometry("900x650")
        self.minsize(760, 480)
        self.configure(bg=BG)
        self.transient(parent)
        self.result = None

        self.files = [Path(p) for p in files]
        self.slot_vars = []

        try:
            first_slot = int(start_slot)
        except Exception:
            first_slot = 1
        first_slot = max(1, min(100, first_slot))

        outer = ttk.Frame(self, padding=16, style="App.TFrame")
        outer.pack(fill="both", expand=True)

        ttk.Label(
            outer,
            text="BATCH TRAJECTORY UPLOAD",
            style="Title.TLabel",
        ).pack(anchor="w")

        ttk.Label(
            outer,
            text=(
                "Set one controller Program ID for every file. "
                "The IDs below are prefilled sequentially and can be edited."
            ),
            style="Muted.TLabel",
        ).pack(anchor="w", pady=(4, 12))

        header = ttk.Frame(outer, style="Panel.TFrame", padding=(10, 7))
        header.pack(fill="x")

        ttk.Label(
            header, text="#", style="StatusTitle.TLabel", width=5
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            header, text="TRAJECTORY FILE", style="StatusTitle.TLabel"
        ).grid(row=0, column=1, sticky="w")
        ttk.Label(
            header, text="PROGRAM ID", style="StatusTitle.TLabel"
        ).grid(row=0, column=2, sticky="e", padx=(20, 6))
        header.columnconfigure(1, weight=1)

        body_holder = ttk.Frame(outer, style="App.TFrame")
        body_holder.pack(fill="both", expand=True, pady=(4, 10))

        canvas = tk.Canvas(
            body_holder,
            bg=BG,
            highlightthickness=0,
            borderwidth=0,
        )
        scrollbar = ttk.Scrollbar(
            body_holder,
            orient="vertical",
            command=canvas.yview,
        )
        canvas.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        rows = ttk.Frame(canvas, style="App.TFrame")
        window_id = canvas.create_window((0, 0), window=rows, anchor="nw")

        def sync_scrollregion(_event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def sync_width(event):
            canvas.itemconfigure(window_id, width=event.width)

        rows.bind("<Configure>", sync_scrollregion)
        canvas.bind("<Configure>", sync_width)

        for i, path in enumerate(self.files):
            row = ttk.Frame(rows, padding=(10, 7), style="Panel.TFrame")
            row.pack(fill="x", pady=2)

            ttk.Label(
                row,
                text=f"{i + 1:02d}",
                style="PanelMuted.TLabel",
                width=5,
            ).grid(row=0, column=0, sticky="w")

            ttk.Label(
                row,
                text=path.name,
                style="Panel.TLabel",
            ).grid(row=0, column=1, sticky="w")

            default_slot = first_slot + i
            if default_slot > 100:
                default_slot = 100

            var = tk.StringVar(value=str(default_slot))
            self.slot_vars.append(var)

            spin = tk.Spinbox(
                row,
                from_=1,
                to=100,
                textvariable=var,
                width=8,
                bg=ENTRY_BG,
                fg=TEXT,
                insertbackground="#FFFFFF",
                insertwidth=3,
                selectbackground=BLUE,
                selectforeground="#FFFFFF",
                buttonbackground=PANEL_2,
                relief="flat",
                highlightthickness=1,
                highlightbackground=BORDER,
                highlightcolor=CYAN,
                font=("DejaVu Sans", 10),
            )
            spin.grid(row=0, column=2, sticky="e", padx=(20, 6), ipady=4)

            row.columnconfigure(1, weight=1)

        controls = ttk.Frame(outer, style="App.TFrame")
        controls.pack(fill="x")

        ttk.Label(
            controls,
            text=(
                "Before each upload, the controller slot will be checked. "
                "Existing programs will be shown before replacement."
            ),
            style="Muted.TLabel",
        ).pack(side="left", fill="x", expand=True)

        ttk.Button(
            controls,
            text="⬆⬆  UPLOAD ALL",
            command=self._accept,
            style="Upload.TButton",
        ).pack(side="right", padx=(8, 0))

        ttk.Button(
            controls,
            text="CANCEL",
            command=self._cancel,
        ).pack(side="right")

        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self.bind("<Escape>", lambda _e: self._cancel())
        self.grab_set()

    def _accept(self):
        jobs = []
        used = {}

        for index, (path, var) in enumerate(zip(self.files, self.slot_vars), 1):
            raw = var.get().strip()
            try:
                slot = int(raw)
            except ValueError:
                messagebox.showerror(
                    "Program ID",
                    f"{path.name}: Program ID must be a number from 1 to 100.",
                    parent=self,
                )
                return

            if not 1 <= slot <= 100:
                messagebox.showerror(
                    "Program ID",
                    f"{path.name}: Program ID {slot} is outside 1–100.",
                    parent=self,
                )
                return

            if slot in used:
                messagebox.showerror(
                    "Duplicate Program ID",
                    f"Program ID {slot} is assigned to both:\n\n"
                    f"• {used[slot].name}\n"
                    f"• {path.name}\n\n"
                    "Each file in the batch must use a unique Program ID.",
                    parent=self,
                )
                return

            used[slot] = path
            jobs.append((path, slot))

        self.result = jobs
        self.destroy()

    def _cancel(self):
        self.result = None
        self.destroy()

    def show(self):
        self.wait_window()
        return self.result


class MultiRunDialog(tk.Toplevel):
    """
    Table-based sequence editor.

    Editable per-row columns:
      Hand
      Pole (mm)
      Wait (s)

    Double-click an editable cell, type the value, then press Enter.
    """

    EDITABLE_COLUMNS = {"hand", "pole", "wait"}

    def __init__(self, parent, programs):
        super().__init__(parent)
        self.title("Multi Run // Sequence Table")
        self.geometry("1260x720")
        self.minsize(980, 580)
        self.configure(bg=BG)
        self.transient(parent)

        self.programs = dict(programs)
        self.queue = []
        self.result = None
        self._cell_editor = None

        self.sequence_dir = Path.home() / "Downloads" / "RealMan_Sequences"
        try:
            self.sequence_dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            self.sequence_dir = Path.home()

        self.search_var = tk.StringVar()
        self.quick_var = tk.StringVar()
        self.count_var = tk.StringVar(value="0 programs in sequence")

        outer = ttk.Frame(self, padding=16, style="App.TFrame")
        outer.pack(fill="both", expand=True)

        ttk.Label(
            outer,
            text="MULTI RUN // SEQUENCE TABLE",
            style="Title.TLabel",
        ).pack(anchor="w")

        ttk.Label(
            outer,
            text=(
                "Add programs, then edit Hand / Pole / Wait directly in the table. "
                "Double-click a value cell and type."
            ),
            style="Muted.TLabel",
        ).pack(anchor="w", pady=(4, 12))

        body = ttk.Frame(outer, style="App.TFrame")
        body.pack(fill="both", expand=True)

        # ------------------------------------------------------------------
        # Available programs
        # ------------------------------------------------------------------
        left = ttk.LabelFrame(
            body,
            text="AVAILABLE PROGRAMS",
            padding=10,
            style="Tech.TLabelframe",
        )
        left.pack(side="left", fill="both", padx=(0, 8))

        search = ttk.Frame(left, style="Panel.TFrame")
        search.pack(fill="x", pady=(0, 8))

        ttk.Label(
            search,
            text="Search",
            style="Panel.TLabel",
        ).pack(side="left", padx=(0, 6))

        self.search_entry = tk.Entry(
            search,
            textvariable=self.search_var,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 10),
            width=28,
        )
        self.search_entry.pack(side="left", fill="x", expand=True, ipady=5)

        av_holder = ttk.Frame(left, style="Panel.TFrame")
        av_holder.pack(fill="both", expand=True)

        self.available = tk.Listbox(
            av_holder,
            selectmode="extended",
            width=37,
            bg=ENTRY_BG,
            fg=TEXT,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            activestyle="none",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans Mono", 10),
        )
        self.available.pack(side="left", fill="both", expand=True)

        av_scroll = ttk.Scrollbar(
            av_holder,
            orient="vertical",
            command=self.available.yview,
        )
        av_scroll.pack(side="right", fill="y")
        self.available.configure(yscrollcommand=av_scroll.set)

        addbar = ttk.Frame(left, style="Panel.TFrame")
        addbar.pack(fill="x", pady=(8, 0))

        ttk.Button(
            addbar,
            text="ADD SELECTED →",
            command=self._add_selected,
            style="Run.TButton",
        ).pack(fill="x")

        # ------------------------------------------------------------------
        # Sequence table
        # ------------------------------------------------------------------
        right = ttk.LabelFrame(
            body,
            text="RUN SEQUENCE // DOUBLE-CLICK HAND, POLE OR WAIT TO EDIT",
            padding=10,
            style="Tech.TLabelframe",
        )
        right.pack(side="left", fill="both", expand=True, padx=(8, 0))

        table_holder = ttk.Frame(right, style="Panel.TFrame")
        table_holder.pack(fill="both", expand=True)

        columns = ("order", "program_id", "name", "hand", "pole", "wait")
        self.table = ttk.Treeview(
            table_holder,
            columns=columns,
            show="headings",
            selectmode="browse",
        )

        self.table.heading("order", text="#")
        self.table.heading("program_id", text="Program ID")
        self.table.heading("name", text="Program Name")
        self.table.heading("hand", text="Hand")
        self.table.heading("pole", text="Pole (mm)")
        self.table.heading("wait", text="Wait (s)")

        self.table.column("order", width=45, minwidth=40, anchor="center", stretch=False)
        self.table.column("program_id", width=90, minwidth=80, anchor="center", stretch=False)
        self.table.column("name", width=340, minwidth=180, anchor="w", stretch=True)
        self.table.column("hand", width=190, minwidth=120, anchor="center", stretch=True)
        self.table.column("pole", width=100, minwidth=85, anchor="center", stretch=False)
        self.table.column("wait", width=90, minwidth=80, anchor="center", stretch=False)

        ybar = ttk.Scrollbar(
            table_holder,
            orient="vertical",
            command=self.table.yview,
        )
        xbar = ttk.Scrollbar(
            table_holder,
            orient="horizontal",
            command=self.table.xview,
        )
        self.table.configure(
            yscrollcommand=ybar.set,
            xscrollcommand=xbar.set,
        )

        self.table.grid(row=0, column=0, sticky="nsew")
        ybar.grid(row=0, column=1, sticky="ns")
        xbar.grid(row=1, column=0, sticky="ew")
        table_holder.rowconfigure(0, weight=1)
        table_holder.columnconfigure(0, weight=1)

        self.table.bind("<Double-1>", self._begin_cell_edit)
        self.table.bind("<Delete>", lambda _e: self._remove_selected())
        self.table.bind("<Button-1>", self._commit_editor_before_click, add="+")

        row_tools = ttk.Frame(right, style="Panel.TFrame")
        row_tools.pack(fill="x", pady=(8, 0))

        ttk.Button(
            row_tools,
            text="↑ UP",
            command=self._move_up,
        ).pack(side="left", padx=(0, 5))

        ttk.Button(
            row_tools,
            text="↓ DOWN",
            command=self._move_down,
        ).pack(side="left", padx=5)

        ttk.Button(
            row_tools,
            text="REMOVE",
            command=self._remove_selected,
            style="Stop.TButton",
        ).pack(side="left", padx=5)

        ttk.Button(
            row_tools,
            text="CLEAR",
            command=self._clear,
        ).pack(side="right")

        # Help
        help_box = ttk.Frame(right, style="Panel.TFrame")
        help_box.pack(fill="x", pady=(10, 0))

        ttk.Label(
            help_box,
            text=(
                "Hand: blank = KEEP current • 50 or 50% = first 4 fingers 50% open "
                "(thumb channels hold) • or six counts: 1000,1000,1000,1000,500,500"
            ),
            style="PanelMuted.TLabel",
            wraplength=760,
        ).pack(anchor="w")

        ttk.Label(
            help_box,
            text=(
                "Pole: blank = KEEP current • 0–315 mm   |   "
                "Wait: blank/0 = no delay • seconds, e.g. 0.5, 2, 5"
            ),
            style="PanelMuted.TLabel",
        ).pack(anchor="w", pady=(3, 0))

        ttk.Label(
            help_box,
            text="Execution for each row: POLE → HAND → WAIT → TRAJECTORY",
            style="StatusTitle.TLabel",
        ).pack(anchor="w", pady=(5, 0))

        # ------------------------------------------------------------------
        # Quick add + footer
        # ------------------------------------------------------------------
        quick = ttk.Frame(outer, style="App.TFrame")
        quick.pack(fill="x", pady=(12, 0))

        ttk.Label(
            quick,
            text="Quick add IDs/names",
            style="Muted.TLabel",
        ).pack(side="left", padx=(0, 6))

        self.quick_entry = tk.Entry(
            quick,
            textvariable=self.quick_var,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 10),
        )
        self.quick_entry.pack(side="left", fill="x", expand=True, ipady=5)
        self.quick_entry.bind("<Return>", lambda _e: self._quick_add())

        ttk.Button(
            quick,
            text="ADD",
            command=self._quick_add,
            style="List.TButton",
        ).pack(side="left", padx=(6, 0))

        footer = ttk.Frame(outer, style="App.TFrame")
        footer.pack(fill="x", pady=(10, 0))

        ttk.Label(
            footer,
            textvariable=self.count_var,
            style="Muted.TLabel",
        ).pack(side="left")

        ttk.Button(
            footer,
            text="▶▶  RUN SEQUENCE",
            command=self._accept,
            style="Run.TButton",
        ).pack(side="right", padx=(8, 0))

        ttk.Button(
            footer,
            text="💾  SAVE SEQUENCE",
            command=self._save_sequence,
            style="Upload.TButton",
        ).pack(side="right", padx=(8, 0))

        ttk.Button(
            footer,
            text="📂  LOAD SEQUENCE",
            command=self._load_sequence,
            style="List.TButton",
        ).pack(side="right", padx=(8, 0))

        ttk.Button(
            footer,
            text="CANCEL",
            command=self._cancel,
        ).pack(side="right")

        self.search_var.trace_add("write", lambda *_: self._refresh_available())
        self.available.bind("<Double-1>", lambda _e: self._add_selected())

        self._refresh_available()
        self._refresh_table()

        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self.bind("<Escape>", lambda _e: self._cancel())
        self.grab_set()
        self.search_entry.focus_set()

    # ------------------------------------------------------------------
    # Data normalization
    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_hand(raw):
        s = str(raw or "").strip()
        if not s:
            return None

        if "," in s:
            parts = [p.strip() for p in s.split(",")]
            if len(parts) != 6:
                raise ValueError("Hand needs exactly 6 comma-separated channel values.")
            values = [int(p) for p in parts]
            if any(v < 0 or v > 1000 for v in values):
                raise ValueError("Every Hand channel must be 0–1000.")
            return ",".join(str(v) for v in values)

        if s.endswith("%"):
            s = s[:-1].strip()

        pct = float(s)
        if not 0 <= pct <= 100:
            raise ValueError("Hand percentage must be 0–100.")
        return f"{pct:g}%"

    @staticmethod
    def _normalize_pole(raw):
        s = str(raw or "").strip()
        if not s:
            return None
        mm = float(s)
        if not 0 <= mm <= 315:
            raise ValueError("Pole must be 0–315 mm.")
        return float(mm)

    @staticmethod
    def _normalize_wait(raw):
        s = str(raw or "").strip()
        if not s:
            return 0.0
        seconds = float(s)
        if not 0 <= seconds <= 3600:
            raise ValueError("Wait must be 0–3600 seconds.")
        return float(seconds)

    def _new_step(self, sid, hand=None, pole_mm=None, wait_s=0.0):
        sid = int(sid)
        return {
            "id": sid,
            "name": self.programs.get(sid, ""),
            "hand": hand,
            "pole_mm": pole_mm,
            "wait_s": float(wait_s or 0.0),
        }

    # ------------------------------------------------------------------
    # Available-program list
    # ------------------------------------------------------------------
    def _refresh_available(self):
        self.available.delete(0, "end")
        query = self.search_var.get().strip().casefold()

        for sid, name in sorted(self.programs.items()):
            display_name = name or "(unnamed)"
            if "DT_" in str(name or "").upper():
                display_name = f"[DRAG TEACH] {display_name}"
            display = f"{sid:03d}  |  {display_name}"
            if query and query not in display.casefold():
                continue
            self.available.insert("end", display)

    def _add_selected(self):
        for idx in self.available.curselection():
            display = self.available.get(idx)
            m = re.match(r"^\s*(\d{1,3})", display)
            if m:
                self.queue.append(self._new_step(int(m.group(1))))

        if self.queue:
            self._refresh_table(select_index=len(self.queue) - 1)

    def _resolve_program(self, token):
        token = token.strip()
        if not token:
            return None

        if token.isdigit():
            sid = int(token)
            if sid in self.programs:
                return sid
            messagebox.showerror(
                "Program not found",
                f"Program ID {sid} is not in the loaded controller list.",
                parent=self,
            )
            return None

        wanted = token.casefold()

        exact = [
            sid for sid, name in self.programs.items()
            if name and name.casefold() == wanted
        ]
        if len(exact) == 1:
            return exact[0]

        partial = [
            sid for sid, name in self.programs.items()
            if name and wanted in name.casefold()
        ]
        if len(partial) == 1:
            return partial[0]

        if len(partial) > 1:
            details = "\n".join(
                f"{sid}: {self.programs[sid]}"
                for sid in sorted(partial)[:10]
            )
            messagebox.showerror(
                "Multiple matches",
                f'"{token}" matches multiple programs:\n\n{details}\n\n'
                "Use the exact name or Program ID.",
                parent=self,
            )
        else:
            messagebox.showerror(
                "Program not found",
                f'No stored controller program matches "{token}".',
                parent=self,
            )
        return None

    def _quick_add(self):
        raw = self.quick_var.get().strip()
        if not raw:
            return

        added = 0
        for token in [
            t.strip()
            for t in re.split(r"[,;\n]+", raw)
            if t.strip()
        ]:
            sid = self._resolve_program(token)
            if sid is not None:
                self.queue.append(self._new_step(sid))
                added += 1

        if added:
            self.quick_var.set("")
            self._refresh_table(select_index=len(self.queue) - 1)

    # ------------------------------------------------------------------
    # Sequence table
    # ------------------------------------------------------------------
    def _row_values(self, index, step):
        hand = step.get("hand") or ""
        pole = step.get("pole_mm")
        wait_s = float(step.get("wait_s") or 0.0)

        return (
            index + 1,
            step["id"],
            self.programs.get(step["id"], step.get("name", "")) or "(unnamed)",
            hand,
            "" if pole is None else f"{float(pole):g}",
            "" if wait_s == 0 else f"{wait_s:g}",
        )

    def _refresh_table(self, select_index=None):
        self._destroy_cell_editor(commit=True)

        for item in self.table.get_children():
            self.table.delete(item)

        for i, step in enumerate(self.queue):
            self.table.insert(
                "",
                "end",
                iid=f"row_{i}",
                values=self._row_values(i, step),
            )

        count = len(self.queue)
        self.count_var.set(
            f"{count} program{'s' if count != 1 else ''} in sequence"
        )

        if select_index is not None and self.queue:
            select_index = max(0, min(select_index, len(self.queue) - 1))
            iid = f"row_{select_index}"
            self.table.selection_set(iid)
            self.table.focus(iid)
            self.table.see(iid)

    def _selected_index(self):
        selection = self.table.selection()
        if not selection:
            return None
        m = re.match(r"row_(\d+)$", selection[0])
        return int(m.group(1)) if m else None

    def _remove_selected(self):
        idx = self._selected_index()
        if idx is None:
            return
        self._destroy_cell_editor(commit=True)
        del self.queue[idx]
        self._refresh_table(
            select_index=min(idx, len(self.queue) - 1) if self.queue else None
        )

    def _move_up(self):
        idx = self._selected_index()
        if idx is None or idx == 0:
            return
        self._destroy_cell_editor(commit=True)
        self.queue[idx - 1], self.queue[idx] = self.queue[idx], self.queue[idx - 1]
        self._refresh_table(select_index=idx - 1)

    def _move_down(self):
        idx = self._selected_index()
        if idx is None or idx >= len(self.queue) - 1:
            return
        self._destroy_cell_editor(commit=True)
        self.queue[idx + 1], self.queue[idx] = self.queue[idx], self.queue[idx + 1]
        self._refresh_table(select_index=idx + 1)

    def _clear(self):
        self._destroy_cell_editor(commit=False)
        self.queue.clear()
        self._refresh_table()

    # ------------------------------------------------------------------
    # Inline cell editing
    # ------------------------------------------------------------------
    def _column_name_from_id(self, column_id):
        # Treeview columns are #1..#6.
        mapping = {
            "#1": "order",
            "#2": "program_id",
            "#3": "name",
            "#4": "hand",
            "#5": "pole",
            "#6": "wait",
        }
        return mapping.get(column_id)

    def _begin_cell_edit(self, event):
        self._destroy_cell_editor(commit=True)

        region = self.table.identify_region(event.x, event.y)
        if region != "cell":
            return

        row_id = self.table.identify_row(event.y)
        column_id = self.table.identify_column(event.x)
        column_name = self._column_name_from_id(column_id)

        if not row_id or column_name not in self.EDITABLE_COLUMNS:
            return

        m = re.match(r"row_(\d+)$", row_id)
        if not m:
            return

        row_index = int(m.group(1))
        bbox = self.table.bbox(row_id, column_id)
        if not bbox:
            return

        x, y, width, height = bbox

        current = ""
        step = self.queue[row_index]
        if column_name == "hand":
            current = step.get("hand") or ""
        elif column_name == "pole":
            current = "" if step.get("pole_mm") is None else f"{float(step['pole_mm']):g}"
        elif column_name == "wait":
            wait_s = float(step.get("wait_s") or 0.0)
            current = "" if wait_s == 0 else f"{wait_s:g}"

        editor = tk.Entry(
            self.table,
            bg="#102B42",
            fg="#FFFFFF",
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="solid",
            bd=1,
            highlightthickness=1,
            highlightbackground=CYAN,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 10, "bold"),
        )
        editor.insert(0, current)
        editor.select_range(0, "end")
        editor.place(x=x, y=y, width=width, height=height)
        editor.focus_set()

        self._cell_editor = {
            "widget": editor,
            "row": row_index,
            "column": column_name,
        }

        editor.bind("<Return>", lambda _e: self._destroy_cell_editor(commit=True))
        editor.bind("<Escape>", lambda _e: self._destroy_cell_editor(commit=False))
        editor.bind("<FocusOut>", lambda _e: self._destroy_cell_editor(commit=True))

    def _commit_editor_before_click(self, _event=None):
        # Clicking elsewhere commits the current edited value first.
        if self._cell_editor is not None:
            self._destroy_cell_editor(commit=True)

    def _destroy_cell_editor(self, commit):
        info = self._cell_editor
        if info is None:
            return True

        self._cell_editor = None
        widget = info["widget"]

        try:
            value = widget.get()
        except Exception:
            value = ""

        try:
            widget.destroy()
        except Exception:
            pass

        if not commit:
            return True

        row = info["row"]
        column = info["column"]

        if not (0 <= row < len(self.queue)):
            return True

        try:
            if column == "hand":
                self.queue[row]["hand"] = self._normalize_hand(value)
            elif column == "pole":
                self.queue[row]["pole_mm"] = self._normalize_pole(value)
            elif column == "wait":
                self.queue[row]["wait_s"] = self._normalize_wait(value)
        except Exception as exc:
            messagebox.showerror(
                "Invalid table value",
                str(exc),
                parent=self,
            )
            # Re-open the same cell with the old valid value.
            self._refresh_table(select_index=row)
            return False

        # Update just this row visually.
        iid = f"row_{row}"
        if self.table.exists(iid):
            self.table.item(
                iid,
                values=self._row_values(row, self.queue[row]),
            )
            self.table.selection_set(iid)
            self.table.focus(iid)

        return True

    # ------------------------------------------------------------------
    # Save / Load
    # ------------------------------------------------------------------
    def _sequence_data(self):
        if not self._destroy_cell_editor(commit=True):
            return None

        return {
            "format": "realman_multi_run_sequence",
            "version": 3,
            "programs": [
                {
                    "id": int(step["id"]),
                    "name": self.programs.get(step["id"], ""),
                    "hand": step.get("hand"),
                    "pole_mm": step.get("pole_mm"),
                    "wait_s": float(step.get("wait_s") or 0.0),
                }
                for step in self.queue
            ],
        }

    def _save_sequence(self):
        if not self.queue:
            messagebox.showinfo(
                "Save Sequence",
                "Add at least one program before saving.",
                parent=self,
            )
            return

        data = self._sequence_data()
        if not data:
            return

        path = filedialog.asksaveasfilename(
            title="Save RealMan Multi Run sequence",
            initialdir=str(self.sequence_dir),
            initialfile="realman_sequence.json",
            defaultextension=".json",
            filetypes=[
                ("RealMan sequence", "*.json"),
                ("All files", "*.*"),
            ],
            parent=self,
        )
        if not path:
            return

        try:
            Path(path).write_text(
                json.dumps(data, indent=2),
                encoding="utf-8",
            )
        except OSError as exc:
            messagebox.showerror(
                "Save Sequence",
                f"Could not save the sequence:\n\n{exc}",
                parent=self,
            )
            return

        messagebox.showinfo(
            "Sequence Saved",
            f"Saved {len(self.queue)} rows with Hand / Pole / Wait values:\n\n{path}",
            parent=self,
        )

    def _read_sequence_file(self, path):
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except Exception as exc:
            messagebox.showerror(
                "Load Sequence",
                f"Could not read the sequence file:\n\n{exc}",
                parent=self,
            )
            return None

        if not isinstance(data, dict) or data.get("format") != "realman_multi_run_sequence":
            messagebox.showerror(
                "Load Sequence",
                "This file is not a valid RealMan Multi Run sequence.",
                parent=self,
            )
            return None

        raw = data.get("programs")
        if not isinstance(raw, list) or not raw:
            messagebox.showerror(
                "Load Sequence",
                "The sequence contains no programs.",
                parent=self,
            )
            return None

        steps = []
        try:
            for item in raw:
                if not isinstance(item, dict):
                    continue

                sid = int(item.get("id"))
                if not 1 <= sid <= 100:
                    continue

                steps.append({
                    "id": sid,
                    "name": str(item.get("name") or ""),
                    "hand": self._normalize_hand(item.get("hand")),
                    "pole_mm": self._normalize_pole(item.get("pole_mm")),
                    "wait_s": self._normalize_wait(
                        item.get("wait_s", item.get("wait"))
                    ),
                })
        except Exception as exc:
            messagebox.showerror(
                "Load Sequence",
                f"Invalid Hand / Pole / Wait value:\n\n{exc}",
                parent=self,
            )
            return None

        if not steps:
            messagebox.showerror(
                "Load Sequence",
                "No valid Program IDs were found.",
                parent=self,
            )
            return None

        return steps

    def _validate_loaded_sequence(self, steps):
        missing = []
        renamed = []
        validated = []

        for step in steps:
            sid = step["id"]
            saved_name = step.get("name", "")

            if sid not in self.programs:
                missing.append((sid, saved_name))
                continue

            current_name = self.programs.get(sid, "")
            if saved_name and current_name and saved_name != current_name:
                renamed.append((sid, saved_name, current_name))

            updated = dict(step)
            updated["name"] = current_name
            validated.append(updated)

        if missing:
            details = "\n".join(
                f"{sid}: {name or '(unnamed)'}"
                for sid, name in missing[:12]
            )
            messagebox.showerror(
                "Sequence Cannot Be Loaded",
                "These Program IDs are not currently on the controller:\n\n"
                f"{details}",
                parent=self,
            )
            return None

        if renamed:
            details = "\n\n".join(
                f"Program {sid}\nSaved:   {old or '(unnamed)'}\n"
                f"Current: {new or '(unnamed)'}"
                for sid, old, new in renamed[:10]
            )
            use_current = messagebox.askyesno(
                "Stored Programs Changed",
                "Some Program IDs contain different names now:\n\n"
                f"{details}\n\n"
                "Use the CURRENT programs with the saved Hand / Pole / Wait values?",
                icon="warning",
                parent=self,
            )
            if not use_current:
                return None

        return validated

    def _load_sequence(self):
        path = filedialog.askopenfilename(
            title="Load RealMan Multi Run sequence",
            initialdir=str(self.sequence_dir),
            filetypes=[
                ("RealMan sequence", "*.json"),
                ("All files", "*.*"),
            ],
            parent=self,
        )
        if not path:
            return

        steps = self._read_sequence_file(path)
        if not steps:
            return

        validated = self._validate_loaded_sequence(steps)
        if not validated:
            return

        self.queue = validated
        self._refresh_table(select_index=0)

        self.count_var.set(
            f"{len(self.queue)} program{'s' if len(self.queue) != 1 else ''} "
            "loaded with Hand / Pole / Wait values"
        )

    def _accept(self):
        if not self.queue:
            messagebox.showinfo(
                "Run Sequence",
                "Add at least one stored controller program.",
                parent=self,
            )
            return

        if not self._destroy_cell_editor(commit=True):
            return

        self.result = [dict(step) for step in self.queue]
        self.destroy()

    def _cancel(self):
        self._destroy_cell_editor(commit=False)
        self.result = None
        self.destroy()

    def show(self):
        self.wait_window()
        return self.result



class ArmControlPanel(ttk.Frame):
    """
    Task-oriented control surface for one RealMan controller.

    Shared from the main window:
      - rm_upload.py path
      - SDK path
      - trajectory file
      - upload slot

    Independent per arm:
      - commands/process
      - stored-program list
      - sequence execution
      - Hand / Pole control
      - status/runtime
      - telemetry
    """

    def __init__(
        self,
        parent,
        *,
        side,
        ip_address,
        rm_upload_var,
        sdk_var,
        track_var,
        slot_var,
        default_track_dir,
    ):
        super().__init__(parent, style="App.TFrame")

        self.side_name = side
        self.ip_address = ip_address

        # Shared setup variables.
        self.rm_upload_var = rm_upload_var
        self.sdk_var = sdk_var
        self.track_var = track_var
        self.slot_var = slot_var
        self.default_track_dir = default_track_dir

        # Fixed side for this half of the UI.
        self.side_var = tk.StringVar(value=side)

        # Independent command/process state.
        self.process = None
        self.worker = None
        # Authoritative result of the most recently launched controller command.
        # None = not finished / not launched yet; 0 = success; non-zero = fault.
        self.last_command_rc = None
        self.motion_capable_command = False
        self.batch_active = False
        self.multi_run_active = False
        self.sequence_cancel_requested = False
        self.sequence_started_at = None

        # Independent controller program cache.
        self.controller_programs = {}

        self.stored_program_var = tk.StringVar()
        self.program_count_var = tk.StringVar(value="Program list not loaded")
        self.status_var = tk.StringVar(
            value=f"READY // {side.upper()} arm waiting for command"
        )
        self.runtime_var = tk.StringVar(value="0.0 s")
        self.arm_mode_var = tk.StringVar(value="MODE // UNKNOWN")

        # Controller global waypoint state.
        self.global_points = {}
        self.global_visible_names = []
        self.global_point_sequence = []
        self.global_search_var = tk.StringVar()
        self.global_count_var = tk.StringVar(value="Global points not loaded")
        self.global_speed_var = tk.StringVar(value="20")
        self.global_wait_var = tk.StringVar(value="0")
        self._global_cell_editor = None

        # Independent auxiliary controls.
        # Inspire hand native order:
        # Little, Ring, Middle, Index, Thumb Bend, Thumb Rotate.
        self.hand_channel_names = (
            "Little",
            "Ring",
            "Middle",
            "Index",
            "Thumb Bend",
            "Thumb Rotate",
        )
        self.manual_hand_vars = [
            tk.IntVar(value=1000) for _ in self.hand_channel_names
        ]
        self.manual_hand_entry_vars = [
            tk.StringVar(value="1000") for _ in self.hand_channel_names
        ]
        self.manual_hand_sliders = []
        self.manual_hand_entries = []

        self.manual_pole_var = tk.DoubleVar(value=0.0)
        self.manual_pole_text_var = tk.StringVar(value="0 mm")

        self.run_started_at = None
        self.runtime_after_id = None

        # Drag & Teach test state.
        self.drag_teach_active = False
        self.drag_teach_has_recording = False
        self.drag_save_var = tk.StringVar(value="")

        self._build_panel_ui()

    def _build_panel_ui(self):
        """Task-oriented, side-by-side arm console."""
        outer = ttk.Frame(
            self,
            padding=(8, 8, 8, 8),
            style="App.TFrame",
        )
        outer.pack(fill="both", expand=True)

        # Compatibility with older page-scroll helper; this layout uses tabs.
        self.scroll_canvas = None

        # --------------------------------------------------------------
        # Always-visible arm identity + run mode + stop
        # --------------------------------------------------------------
        arm_header = ttk.Frame(
            outer,
            padding=(10, 8),
            style="Panel2.TFrame",
        )
        arm_header.pack(fill="x", pady=(0, 6))

        identity = ttk.Frame(arm_header, style="Panel2.TFrame")
        identity.pack(side="left", fill="x", expand=True)

        ttk.Label(
            identity,
            text=f"{self.side_name.upper()} ARM",
            style="ArmTitle.TLabel",
        ).pack(side="left")

        ttk.Label(
            identity,
            text=f"// {self.ip_address}",
            style="ArmIp.TLabel",
        ).pack(side="left", padx=(8, 0))

        ttk.Label(
            identity,
            textvariable=self.arm_mode_var,
            style="ModeBadge.TLabel",
        ).pack(side="left", padx=(14, 0))

        mode_tools = ttk.Frame(arm_header, style="Panel2.TFrame")
        mode_tools.pack(side="right")

        self.sim_mode_btn = ttk.Button(
            mode_tools,
            text="SIM",
            command=lambda: self._set_arm_mode_ui("simulation"),
            style="CompactBlue.TButton",
        )
        self.sim_mode_btn.pack(side="left", padx=2)

        self.real_mode_btn = ttk.Button(
            mode_tools,
            text="REAL",
            command=lambda: self._set_arm_mode_ui("real"),
            style="CompactOrange.TButton",
        )
        self.real_mode_btn.pack(side="left", padx=2)

        self.check_mode_btn = ttk.Button(
            mode_tools,
            text="CHECK",
            command=self._check_arm_mode_ui,
            style="Compact.TButton",
        )
        self.check_mode_btn.pack(side="left", padx=2)

        self.quick_stop_btn = ttk.Button(
            mode_tools,
            text="■ SLOW STOP",
            command=self._slow_stop,
            state="disabled",
            style="CompactStop.TButton",
        )
        self.quick_stop_btn.pack(side="left", padx=(8, 0))

        # --------------------------------------------------------------
        # Always-visible status + runtime
        # --------------------------------------------------------------
        status_frame = ttk.Frame(
            outer,
            padding=(10, 7),
            style="Panel.TFrame",
        )
        status_frame.pack(fill="x", pady=(0, 7))

        ttk.Label(
            status_frame,
            text="STATUS",
            style="StatusTitle.TLabel",
        ).pack(side="left", padx=(0, 8))

        ttk.Label(
            status_frame,
            textvariable=self.status_var,
            style="StatusValue.TLabel",
        ).pack(side="left", fill="x", expand=True)

        ttk.Label(
            status_frame,
            text="RUN",
            style="StatusTitle.TLabel",
        ).pack(side="left", padx=(8, 6))

        ttk.Label(
            status_frame,
            textvariable=self.runtime_var,
            style="RuntimeValue.TLabel",
        ).pack(side="right")

        # --------------------------------------------------------------
        # Workflow tabs
        # --------------------------------------------------------------
        self.arm_notebook = ttk.Notebook(
            outer,
            style="Robot.TNotebook",
        )
        self.arm_notebook.pack(fill="both", expand=True)

        run_tab = ttk.Frame(self.arm_notebook, padding=8, style="App.TFrame")
        points_tab = ttk.Frame(self.arm_notebook, padding=8, style="App.TFrame")
        teach_tab = ttk.Frame(self.arm_notebook, padding=8, style="App.TFrame")
        aux_tab = ttk.Frame(self.arm_notebook, padding=8, style="App.TFrame")
        telemetry_tab = ttk.Frame(self.arm_notebook, padding=8, style="App.TFrame")

        self.arm_notebook.add(run_tab, text="  RUN  ")
        self.arm_notebook.add(points_tab, text="  POINTS  ")
        self.arm_notebook.add(teach_tab, text="  DRAG TEACH  ")
        self.arm_notebook.add(aux_tab, text="  HAND / POLE  ")
        self.arm_notebook.add(telemetry_tab, text="  TELEMETRY  ")

        # --------------------------------------------------------------
        # 01 // Per-arm trajectory setup
        # --------------------------------------------------------------
        arm_setup = ttk.LabelFrame(
            run_tab,
            text="TRAJECTORY FILE + UPLOAD SLOT",
            padding=10,
            style="Tech.TLabelframe",
        )
        arm_setup.pack(fill="x", pady=(0, 10))

        ttk.Label(
            arm_setup,
            text="Trajectory file",
            style="Panel.TLabel",
        ).grid(row=0, column=0, sticky="w", padx=(0, 8), pady=5)

        self.arm_track_entry = tk.Entry(
            arm_setup,
            textvariable=self.track_var,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 9),
        )
        self.arm_track_entry.grid(
            row=0,
            column=1,
            columnspan=2,
            sticky="ew",
            pady=5,
            ipady=4,
        )

        ttk.Button(
            arm_setup,
            text="Browse",
            command=self._browse_arm_track,
        ).grid(row=0, column=3, padx=(8, 0), pady=5)

        ttk.Label(
            arm_setup,
            text="Upload slot",
            style="Panel.TLabel",
        ).grid(row=1, column=0, sticky="w", padx=(0, 8), pady=5)

        self.arm_slot_spin = tk.Spinbox(
            arm_setup,
            from_=1,
            to=100,
            textvariable=self.slot_var,
            width=8,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            buttonbackground=PANEL_2,
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 10, "bold"),
        )
        self.arm_slot_spin.grid(
            row=1,
            column=1,
            sticky="w",
            pady=5,
            ipady=3,
        )

        ttk.Label(
            arm_setup,
            text=f"{self.side_name.upper()} only // {self.ip_address}",
            style="PanelMuted.TLabel",
        ).grid(
            row=1,
            column=2,
            columnspan=2,
            sticky="w",
            padx=(12, 0),
            pady=5,
        )

        arm_setup.columnconfigure(1, weight=1)
        arm_setup.columnconfigure(2, weight=1)

        # --------------------------------------------------------------
        # 02 // Trajectory commands
        # --------------------------------------------------------------
        action_frame = ttk.LabelFrame(
            run_tab,
            text="TRAJECTORY COMMANDS",
            padding=10,
            style="Tech.TLabelframe",
        )
        action_frame.pack(fill="x")

        self.upload_btn = ttk.Button(
            action_frame,
            text="⬆  UPLOAD ONLY",
            command=self._upload_only,
            style="Upload.TButton",
        )
        self.upload_btn.grid(row=0, column=0, padx=(0, 5), pady=5, sticky="ew")

        self.upload_run_btn = ttk.Button(
            action_frame,
            text="▶  UPLOAD + RUN",
            command=self._upload_and_run,
            style="Run.TButton",
        )
        self.upload_run_btn.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        self.run_only_btn = ttk.Button(
            action_frame,
            text="▷  RUN ONLY",
            command=self._run_upload_slot_only,
            style="Orange.TButton",
        )
        self.run_only_btn.grid(row=0, column=2, padx=(5, 0), pady=5, sticky="ew")

        self.multi_upload_btn = ttk.Button(
            action_frame,
            text="⬆⬆  MULTI UPLOAD",
            command=self._multi_upload_setup,
            style="Upload.TButton",
        )
        self.multi_upload_btn.grid(row=1, column=0, padx=(0, 5), pady=5, sticky="ew")

        self.stop_btn = ttk.Button(
            action_frame,
            text="■  SLOW STOP",
            command=self._slow_stop,
            state="disabled",
            style="Stop.TButton",
        )
        self.stop_btn.grid(
            row=1,
            column=2,
            padx=(5, 0),
            pady=5,
            sticky="ew",
        )

        ttk.Label(
            action_frame,
            text="Slow Stop acts immediately — no confirmation.",
            style="PanelMuted.TLabel",
        ).grid(
            row=1,
            column=1,
            padx=5,
            pady=5,
            sticky="w",
        )

        for c in range(3):
            action_frame.columnconfigure(c, weight=1)

        # --------------------------------------------------------------
        # 03 // Controller program launcher
        # --------------------------------------------------------------
        programs = ttk.LabelFrame(
            run_tab,
            text="CONTROLLER PROGRAMS / SEQUENCES",
            padding=10,
            style="Tech.TLabelframe",
        )
        programs.pack(fill="x", pady=(10, 0))

        ttk.Label(
            programs,
            text="Program ID or name",
            style="Panel.TLabel",
        ).grid(row=0, column=0, sticky="w", padx=(0, 8), pady=4)

        self.program_combo = ttk.Combobox(
            programs,
            textvariable=self.stored_program_var,
            state="normal",
            style="Program.TCombobox",
        )
        self.program_combo.grid(
            row=0,
            column=1,
            columnspan=3,
            sticky="ew",
            pady=4,
        )
        self.program_combo.bind("<Return>", lambda _e: self._run_existing())

        self.run_btn = ttk.Button(
            programs,
            text="▶ RUN STORED",
            command=self._run_existing,
            style="Orange.TButton",
        )
        self.run_btn.grid(row=1, column=0, padx=(0, 5), pady=5, sticky="ew")

        self.multi_run_btn = ttk.Button(
            programs,
            text="▶▶ MULTI RUN",
            command=self._multi_run_setup,
            style="Run.TButton",
        )
        self.multi_run_btn.grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        self.load_sequence_btn = ttk.Button(
            programs,
            text="📂▶ LOAD SEQUENCE",
            command=self._load_sequence_and_run,
            style="Upload.TButton",
        )
        self.load_sequence_btn.grid(row=1, column=2, padx=5, pady=5, sticky="ew")

        self.list_btn = ttk.Button(
            programs,
            text="↻ REFRESH LIST",
            command=self._list_slots,
            style="List.TButton",
        )
        self.list_btn.grid(row=1, column=3, padx=(5, 0), pady=5, sticky="ew")

        ttk.Label(
            programs,
            textvariable=self.program_count_var,
            style="PanelMuted.TLabel",
        ).grid(row=2, column=0, columnspan=4, sticky="w", pady=(5, 0))

        ttk.Label(
            programs,
            text=(
                "Program list and sequence execution belong only to this arm."
            ),
            style="PanelMuted.TLabel",
        ).grid(row=3, column=0, columnspan=4, sticky="w", pady=(2, 0))

        for c in range(4):
            programs.columnconfigure(c, weight=1)

        # --------------------------------------------------------------
        # 04 // Controller Global Points
        # --------------------------------------------------------------
        global_frame = ttk.LabelFrame(
            points_tab,
            text="GLOBAL POINTS // MOVEJ + MOVEL + SEQUENCE",
            padding=10,
            style="Tech.TLabelframe",
        )
        global_frame.pack(fill="x", pady=(10, 0))

        gp_top = ttk.Frame(global_frame, style="Panel.TFrame")
        gp_top.pack(fill="x")

        ttk.Label(
            gp_top,
            text="Search",
            style="Panel.TLabel",
        ).pack(side="left", padx=(0, 6))

        self.global_search_entry = tk.Entry(
            gp_top,
            textvariable=self.global_search_var,
            width=22,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 9),
        )
        self.global_search_entry.pack(
            side="left",
            fill="x",
            expand=True,
            ipady=4,
        )

        self.global_refresh_btn = ttk.Button(
            gp_top,
            text="↻ REFRESH GLOBAL POINTS",
            command=self._refresh_global_points,
            style="List.TButton",
        )
        self.global_refresh_btn.pack(side="left", padx=(8, 0))

        gp_list_holder = ttk.Frame(global_frame, style="Panel.TFrame")
        gp_list_holder.pack(fill="x", pady=(8, 0))

        self.global_point_list = tk.Listbox(
            gp_list_holder,
            selectmode="extended",
            height=7,
            bg=ENTRY_BG,
            fg=TEXT,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            activestyle="none",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans Mono", 9),
        )
        self.global_point_list.pack(side="left", fill="both", expand=True)

        gp_scroll = ttk.Scrollbar(
            gp_list_holder,
            orient="vertical",
            command=self.global_point_list.yview,
        )
        gp_scroll.pack(side="right", fill="y")
        self.global_point_list.configure(yscrollcommand=gp_scroll.set)

        ttk.Label(
            global_frame,
            textvariable=self.global_count_var,
            style="PanelMuted.TLabel",
        ).pack(anchor="w", pady=(5, 0))

        gp_single = ttk.Frame(global_frame, style="Panel.TFrame")
        gp_single.pack(fill="x", pady=(8, 0))

        ttk.Label(
            gp_single,
            text="Speed %",
            style="Panel.TLabel",
        ).pack(side="left", padx=(0, 5))

        self.global_speed_entry = tk.Entry(
            gp_single,
            textvariable=self.global_speed_var,
            width=6,
            justify="center",
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 9, "bold"),
        )
        self.global_speed_entry.pack(side="left", ipady=3)

        ttk.Label(
            gp_single,
            text="Wait (s)",
            style="Panel.TLabel",
        ).pack(side="left", padx=(10, 5))

        self.global_wait_entry = tk.Entry(
            gp_single,
            textvariable=self.global_wait_var,
            width=7,
            justify="center",
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 9, "bold"),
        )
        self.global_wait_entry.pack(side="left", ipady=3)

        self.global_movej_btn = ttk.Button(
            gp_single,
            text="▶ MOVE J",
            command=lambda: self._run_selected_global_point("movej"),
            style="Orange.TButton",
        )
        self.global_movej_btn.pack(side="left", padx=(10, 4))

        self.global_movel_btn = ttk.Button(
            gp_single,
            text="▶ MOVE L",
            command=lambda: self._run_selected_global_point("movel"),
            style="Run.TButton",
        )
        self.global_movel_btn.pack(side="left", padx=4)

        self.global_sequence_run_btn = ttk.Button(
            gp_single,
            text="▶▶ RUN SEQUENCE",
            command=self._run_global_point_sequence,
            style="Run.TButton",
        )
        self.global_sequence_run_btn.pack(side="left", padx=(4, 4))

        self.global_add_btn = ttk.Button(
            gp_single,
            text="ADD ↓",
            command=self._add_selected_global_points,
            style="Upload.TButton",
        )
        self.global_add_btn.pack(side="right")

        ttk.Label(
            global_frame,
            text=(
                "Single-point Wait executes after arrival. For multiple points, "
                "add them below and edit Motion / Speed / Wait per row."
            ),
            style="PanelMuted.TLabel",
            wraplength=700,
        ).pack(anchor="w", pady=(5, 7))

        gp_seq_holder = ttk.Frame(global_frame, style="Panel.TFrame")
        gp_seq_holder.pack(fill="x")

        gp_columns = ("order", "point", "move", "speed", "wait")
        self.global_sequence_table = ttk.Treeview(
            gp_seq_holder,
            columns=gp_columns,
            show="headings",
            selectmode="browse",
            height=7,
        )
        self.global_sequence_table.heading("order", text="#")
        self.global_sequence_table.heading("point", text="Global Point")
        self.global_sequence_table.heading("move", text="Motion")
        self.global_sequence_table.heading("speed", text="Speed %")
        self.global_sequence_table.heading("wait", text="Wait (s)")

        self.global_sequence_table.column(
            "order", width=38, anchor="center", stretch=False
        )
        self.global_sequence_table.column(
            "point", width=230, anchor="w", stretch=True
        )
        self.global_sequence_table.column(
            "move", width=80, anchor="center", stretch=False
        )
        self.global_sequence_table.column(
            "speed", width=70, anchor="center", stretch=False
        )
        self.global_sequence_table.column(
            "wait", width=70, anchor="center", stretch=False
        )
        self.global_sequence_table.pack(
            side="left",
            fill="x",
            expand=True,
        )

        gp_seq_scroll = ttk.Scrollbar(
            gp_seq_holder,
            orient="vertical",
            command=self.global_sequence_table.yview,
        )
        gp_seq_scroll.pack(side="right", fill="y")
        self.global_sequence_table.configure(
            yscrollcommand=gp_seq_scroll.set
        )

        self.global_sequence_table.bind(
            "<Double-1>",
            self._begin_global_cell_edit,
        )
        self.global_sequence_table.bind(
            "<Delete>",
            lambda _e: self._remove_global_sequence_step(),
        )

        gp_tools = ttk.Frame(global_frame, style="Panel.TFrame")
        gp_tools.pack(fill="x", pady=(7, 0))

        self.global_up_btn = ttk.Button(
            gp_tools,
            text="↑ UP",
            command=self._move_global_sequence_up,
        )
        self.global_up_btn.pack(side="left", padx=(0, 4))

        self.global_down_btn = ttk.Button(
            gp_tools,
            text="↓ DOWN",
            command=self._move_global_sequence_down,
        )
        self.global_down_btn.pack(side="left", padx=4)

        self.global_remove_btn = ttk.Button(
            gp_tools,
            text="REMOVE",
            command=self._remove_global_sequence_step,
            style="Stop.TButton",
        )
        self.global_remove_btn.pack(side="left", padx=4)

        self.global_clear_btn = ttk.Button(
            gp_tools,
            text="CLEAR",
            command=self._clear_global_sequence,
        )
        self.global_clear_btn.pack(side="left", padx=4)

        ttk.Label(
            global_frame,
            text=(
                "Double-click Motion, Speed %, or Wait (s) to edit. "
                "Motion accepts MoveJ or MoveL. Wait occurs AFTER that point "
                "is reached and BEFORE the next point starts."
            ),
            style="PanelMuted.TLabel",
            wraplength=700,
        ).pack(anchor="w", pady=(6, 0))

        self.global_search_var.trace_add(
            "write",
            lambda *_: self._refresh_global_point_list_ui(),
        )

        # --------------------------------------------------------------
        # 05 // Drag & Teach
        # --------------------------------------------------------------
        drag = ttk.LabelFrame(
            teach_tab,
            text="DRAG & TEACH // RECORD + PLAYBACK",
            padding=10,
            style="Tech.TLabelframe",
        )
        drag.pack(fill="x", pady=(10, 8))

        ttk.Label(
            drag,
            text=(
                "Record by hand-guiding, save the trajectory locally, "
                "then move to the recorded start position and play it back."
            ),
            style="PanelMuted.TLabel",
            wraplength=700,
        ).grid(
            row=0,
            column=0,
            columnspan=6,
            sticky="w",
            pady=(0, 7),
        )

        self.drag_toggle_btn = ttk.Button(
            drag,
            text="✋ START DRAG + RECORD",
            command=self._drag_teach_toggle,
            style="Orange.TButton",
        )
        self.drag_toggle_btn.grid(
            row=1,
            column=0,
            columnspan=3,
            sticky="ew",
            padx=(0, 6),
            pady=4,
        )

        self.drag_playback_btn = ttk.Button(
            drag,
            text="▶ MOVE TO START + PLAYBACK",
            command=self._drag_playback,
            state="disabled",
            style="Run.TButton",
        )
        self.drag_playback_btn.grid(
            row=1,
            column=3,
            columnspan=3,
            sticky="ew",
            padx=(6, 0),
            pady=4,
        )

        ttk.Label(
            drag,
            text="Local Drag .txt",
            style="Panel.TLabel",
        ).grid(
            row=2,
            column=0,
            sticky="w",
            padx=(0, 8),
            pady=5,
        )

        self.drag_save_entry = tk.Entry(
            drag,
            textvariable=self.drag_save_var,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 9),
        )
        self.drag_save_entry.grid(
            row=2,
            column=1,
            columnspan=3,
            sticky="ew",
            pady=5,
            ipady=4,
        )

        self.drag_browse_btn = ttk.Button(
            drag,
            text="Choose",
            command=self._browse_drag_save,
        )
        self.drag_browse_btn.grid(
            row=2,
            column=4,
            padx=(8, 5),
            pady=5,
        )

        self.drag_save_btn = ttk.Button(
            drag,
            text="💾 SAVE LOCAL",
            command=self._drag_save,
            state="disabled",
            style="Upload.TButton",
        )
        self.drag_save_btn.grid(
            row=2,
            column=5,
            padx=(5, 0),
            pady=5,
            sticky="ew",
        )

        drag.columnconfigure(1, weight=1)
        drag.columnconfigure(2, weight=1)
        drag.columnconfigure(3, weight=1)

        # --------------------------------------------------------------
        # 05 // Manual Hand / Pole
        # --------------------------------------------------------------
        manual = ttk.LabelFrame(
            aux_tab,
            text="MANUAL HAND / POLE CONTROL",
            padding=10,
            style="Tech.TLabelframe",
        )
        manual.pack(fill="x", pady=(10, 0))

        ttk.Label(
            manual,
            text="HAND // INDIVIDUAL 6-CHANNEL POSITION // 0 → 1000",
            style="Panel.TLabel",
        ).grid(
            row=0,
            column=0,
            columnspan=6,
            sticky="w",
            pady=(2, 4),
        )

        ttk.Label(
            manual,
            text="Move sliders or type values, then press APPLY HAND.",
            style="PanelMuted.TLabel",
        ).grid(
            row=1,
            column=0,
            columnspan=6,
            sticky="w",
            pady=(0, 5),
        )

        hand_slider_frame = ttk.Frame(
            manual,
            style="Panel.TFrame",
        )
        hand_slider_frame.grid(
            row=2,
            column=0,
            columnspan=6,
            sticky="ew",
            pady=(0, 5),
        )

        for col, name in enumerate(self.hand_channel_names):
            hand_slider_frame.columnconfigure(col, weight=1)

            ttk.Label(
                hand_slider_frame,
                text=name,
                style="Panel.TLabel",
                anchor="center",
            ).grid(
                row=0,
                column=col,
                padx=4,
                pady=(0, 3),
            )

            slider = tk.Scale(
                hand_slider_frame,
                from_=1000,
                to=0,
                orient="vertical",
                variable=self.manual_hand_vars[col],
                command=lambda value, idx=col:
                    self._manual_hand_channel_slider_changed(idx, value),
                showvalue=False,
                resolution=1,
                length=190,
                width=14,
                sliderlength=24,
                bg=PANEL,
                fg=TEXT,
                troughcolor="#16A34A",
                activebackground="#7F1D1D",
                highlightthickness=0,
                bd=0,
                relief="flat",
                sliderrelief="raised",
            )
            slider.grid(
                row=1,
                column=col,
                padx=7,
                pady=(0, 3),
            )
            self.manual_hand_sliders.append(slider)

            entry = tk.Entry(
                hand_slider_frame,
                textvariable=self.manual_hand_entry_vars[col],
                width=7,
                justify="center",
                bg=ENTRY_BG,
                fg=TEXT,
                insertbackground="#FFFFFF",
                insertwidth=3,
                selectbackground=BLUE,
                selectforeground="#FFFFFF",
                relief="flat",
                highlightthickness=1,
                highlightbackground=BORDER,
                highlightcolor=CYAN,
                font=("DejaVu Sans", 10, "bold"),
            )
            entry.grid(
                row=2,
                column=col,
                padx=4,
                pady=(2, 2),
                ipady=4,
            )
            entry.bind(
                "<Return>",
                lambda _e, idx=col:
                    self._manual_hand_channel_entry_changed(idx),
            )
            entry.bind(
                "<FocusOut>",
                lambda _e, idx=col:
                    self._manual_hand_channel_entry_changed(idx),
            )
            self.manual_hand_entries.append(entry)

        self.manual_hand_apply_btn = ttk.Button(
            manual,
            text="APPLY HAND // ALL 6 CHANNELS",
            command=self._manual_hand_apply,
            style="Run.TButton",
        )
        self.manual_hand_apply_btn.grid(
            row=3,
            column=0,
            columnspan=6,
            sticky="ew",
            pady=(7, 6),
        )

        hand_preset_buttons = ttk.Frame(
            manual,
            style="Panel.TFrame",
        )
        hand_preset_buttons.grid(
            row=4,
            column=0,
            columnspan=6,
            sticky="ew",
            pady=(0, 8),
        )
        hand_preset_buttons.columnconfigure(0, weight=1)
        hand_preset_buttons.columnconfigure(1, weight=1)

        self.manual_hand_grip_btn = ttk.Button(
            hand_preset_buttons,
            text="GRIP PRESET // 0 0 0 0 150 1000",
            command=lambda: self._manual_hand_apply_preset(
                [0, 0, 0, 0, 150, 1000]
            ),
            style="Orange.TButton",
        )
        self.manual_hand_grip_btn.grid(
            row=0,
            column=0,
            sticky="ew",
            padx=(0, 4),
        )

        self.manual_hand_open_btn = ttk.Button(
            hand_preset_buttons,
            text="ALL OPEN // 1000 1000 1000 1000 1000 1000",
            command=lambda: self._manual_hand_apply_preset(
                [1000, 1000, 1000, 1000, 1000, 1000]
            ),
            style="Run.TButton",
        )
        self.manual_hand_open_btn.grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(4, 0),
        )

        ttk.Separator(
            manual,
            orient="horizontal",
        ).grid(
            row=5,
            column=0,
            columnspan=6,
            sticky="ew",
            pady=(1, 8),
        )

        ttk.Label(
            manual,
            text="POLE // 0 mm → 315 mm",
            style="Panel.TLabel",
        ).grid(
            row=6,
            column=0,
            columnspan=6,
            sticky="w",
            pady=(2, 3),
        )

        self.manual_pole_slider = tk.Scale(
            manual,
            from_=0,
            to=315,
            orient="horizontal",
            variable=self.manual_pole_var,
            command=self._manual_pole_slider_changed,
            showvalue=False,
            resolution=1,
            width=12,
            sliderlength=24,
            bg=PANEL,
            fg=TEXT,
            troughcolor="#16A34A",
            activebackground="#7F1D1D",
            highlightthickness=0,
            bd=0,
            relief="flat",
            sliderrelief="raised",
        )
        self.manual_pole_slider.grid(
            row=7,
            column=0,
            columnspan=5,
            sticky="ew",
            padx=(0, 8),
            pady=(0, 4),
        )

        self.manual_pole_entry_var = tk.StringVar(value="0")
        self.manual_pole_entry = tk.Entry(
            manual,
            textvariable=self.manual_pole_entry_var,
            width=8,
            justify="center",
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 10, "bold"),
        )
        self.manual_pole_entry.grid(
            row=8, column=0, sticky="w", pady=(2, 4), ipady=4
        )
        self.manual_pole_entry.bind(
            "<Return>", lambda _e: self._manual_pole_entry_changed()
        )
        self.manual_pole_entry.bind(
            "<FocusOut>", lambda _e: self._manual_pole_entry_changed()
        )

        ttk.Label(
            manual,
            text="mm",
            style="Panel.TLabel",
        ).grid(row=8, column=1, sticky="w", padx=(4, 8), pady=(2, 4))

        ttk.Label(
            manual,
            textvariable=self.manual_pole_text_var,
            style="PanelMuted.TLabel",
        ).grid(row=8, column=2, sticky="w", padx=(0, 8), pady=(2, 4))

        self.manual_pole_apply_btn = ttk.Button(
            manual,
            text="APPLY POLE",
            command=self._manual_pole_apply,
            style="Upload.TButton",
        )
        self.manual_pole_apply_btn.grid(
            row=8, column=5, sticky="ew", pady=(2, 4)
        )

        ttk.Label(
            manual,
            text=(
                "Green slider track / dark-red handle. "
                "Sliding or typing changes only the target; APPLY sends the command."
            ),
            style="PanelMuted.TLabel",
            wraplength=700,
        ).grid(
            row=9,
            column=0,
            columnspan=6,
            sticky="w",
            pady=(6, 0),
        )

        for c in range(6):
            manual.columnconfigure(c, weight=1)

        # --------------------------------------------------------------
        # 06 // Controller telemetry
        # --------------------------------------------------------------
        log_frame = ttk.LabelFrame(
            telemetry_tab,
            text="CONTROLLER TELEMETRY",
            padding=8,
            style="Tech.TLabelframe",
        )
        log_frame.pack(fill="both", expand=True)

        ttk.Label(
            log_frame,
            text=f"{self.side_name.upper()} controller // {self.ip_address}",
            style="PanelMuted.TLabel",
        ).pack(fill="x", pady=(0, 5))

        log_body = ttk.Frame(log_frame, style="Panel.TFrame")
        log_body.pack(fill="both", expand=True)

        self.log = tk.Text(
            log_body,
            wrap="word",
            height=24,
            state="disabled",
            font=("DejaVu Sans Mono", 9),
            bg=ENTRY_BG,
            fg="#BDF8FF",
            insertbackground=CYAN,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            padx=8,
            pady=8,
        )
        self.log.pack(side="left", fill="both", expand=True)

        log_scroll = ttk.Scrollbar(
            log_body,
            orient="vertical",
            command=self.log.yview,
        )
        log_scroll.pack(side="right", fill="y")
        self.log.configure(yscrollcommand=log_scroll.set)

        ttk.Label(
            telemetry_tab,
            text=(
                "SAFETY // Software Slow Stop is not the hardware E-stop. "
                "Keep the physical E-stop accessible."
            ),
            style="Muted.TLabel",
            wraplength=700,
        ).pack(anchor="w", pady=(8, 0))

    def has_active_command(self):
        if self.multi_run_active or self.batch_active:
            return True
        return bool(self.process and self.process.poll() is None)

    # ------------------------------------------------------------------
    # Global Waypoint UI
    # ------------------------------------------------------------------
    def _refresh_global_points(self):
        if self.has_active_command():
            messagebox.showwarning(
                "Arm busy",
                f"{self.side_name.upper()} arm already has an active command.",
                parent=self.winfo_toplevel(),
            )
            return

        env = self._validate_environment()
        if not env:
            return

        self.global_points.clear()
        self.global_visible_names.clear()
        self._refresh_global_point_list_ui()
        self.global_count_var.set("Reading global points from controller...")

        rm_upload, sdk = env
        cmd = self._base_command(rm_upload, sdk) + [
            "--global-list",
            "--side",
            self.side_name,
            "--yes",
        ]

        self._start_process(
            cmd,
            "GLOBAL POINTS // Reading controller list",
            motion=False,
        )

    def _remember_global_point(self, point):
        if not isinstance(point, dict):
            return

        name = str(point.get("point_name") or "").strip()
        if not name:
            return

        self.global_points[name] = {
            "point_name": name,
            "work_frame": str(point.get("work_frame") or ""),
            "tool_frame": str(point.get("tool_frame") or ""),
            "time": str(point.get("time") or ""),
        }
        self._refresh_global_point_list_ui()

    def _refresh_global_point_list_ui(self):
        if not hasattr(self, "global_point_list"):
            return

        query = self.global_search_var.get().strip().casefold()
        self.global_point_list.delete(0, "end")
        self.global_visible_names = []

        for name in sorted(
            self.global_points,
            key=lambda value: value.casefold(),
        ):
            point = self.global_points[name]
            work = point.get("work_frame") or "-"
            tool = point.get("tool_frame") or "-"
            display = f"{name}  |  W:{work}  |  T:{tool}"

            if query and query not in display.casefold():
                continue

            self.global_visible_names.append(name)
            self.global_point_list.insert("end", display)

        count = len(self.global_points)
        self.global_count_var.set(
            (
                f"{count} global point{'s' if count != 1 else ''} "
                f"loaded from {self.ip_address}"
            )
            if count
            else "No global points loaded"
        )

    def _selected_global_names(self):
        names = []
        for index in self.global_point_list.curselection():
            if 0 <= index < len(self.global_visible_names):
                names.append(self.global_visible_names[index])
        return names

    def _global_speed_wait(self):
        try:
            speed = int(float(self.global_speed_var.get().strip()))
        except Exception:
            messagebox.showerror(
                "Global Point",
                "Speed must be a number from 1 to 100.",
                parent=self.winfo_toplevel(),
            )
            return None

        try:
            wait_s = float(self.global_wait_var.get().strip() or "0")
        except Exception:
            messagebox.showerror(
                "Global Point",
                "Wait must be a number of seconds.",
                parent=self.winfo_toplevel(),
            )
            return None

        if not 1 <= speed <= 100:
            messagebox.showerror(
                "Global Point",
                "Speed must be from 1 to 100.",
                parent=self.winfo_toplevel(),
            )
            return None

        if not 0 <= wait_s <= 3600:
            messagebox.showerror(
                "Global Point",
                "Wait must be from 0 to 3600 seconds.",
                parent=self.winfo_toplevel(),
            )
            return None

        return speed, wait_s

    def _run_selected_global_point(self, move_type):
        if self.has_active_command():
            messagebox.showwarning(
                "Arm busy",
                f"{self.side_name.upper()} arm already has an active command.",
                parent=self.winfo_toplevel(),
            )
            return

        names = self._selected_global_names()
        if len(names) != 1:
            messagebox.showinfo(
                "Global Point",
                "Select exactly one global point for MoveJ / MoveL.",
                parent=self.winfo_toplevel(),
            )
            return

        values = self._global_speed_wait()
        if not values:
            return

        speed, wait_s = values
        name = names[0]
        label = "MoveJ" if move_type == "movej" else "MoveL"

        env = self._validate_environment()
        if not env:
            return

        ok = messagebox.askyesno(
            "Confirm Global Point movement",
            f"{self.side_name.upper()} ARM // {self.ip_address}\n\n"
            f"Global Point: {name}\n"
            f"Motion: {label}\n"
            f"Speed: {speed}%\n"
            f"Wait after arrival: {wait_s:g} s\n\n"
            "THE ROBOT CAN MOVE.\n"
            "Confirm the work area is clear and the hardware E-stop is accessible.",
            icon="warning",
            parent=self.winfo_toplevel(),
        )
        if not ok:
            return

        rm_upload, sdk = env
        cmd = self._base_command(rm_upload, sdk) + [
            "--global-point",
            name,
            "--global-move",
            move_type,
            "--global-speed",
            str(speed),
            "--global-wait",
            str(wait_s),
            "--side",
            self.side_name,
            "--yes",
        ]

        self._start_process(
            cmd,
            f"GLOBAL POINT // {label} // {name}",
            motion=True,
        )

    def _add_selected_global_points(self):
        names = self._selected_global_names()
        if not names:
            messagebox.showinfo(
                "Global Point Sequence",
                "Select one or more global points first.",
                parent=self.winfo_toplevel(),
            )
            return

        values = self._global_speed_wait()
        if not values:
            return

        speed, wait_s = values

        for name in names:
            self.global_point_sequence.append(
                {
                    "name": name,
                    "move": "movej",
                    "speed": speed,
                    "wait_s": wait_s,
                }
            )

        self._refresh_global_sequence_table(
            select_index=len(self.global_point_sequence) - 1
        )

    def _global_sequence_row_values(self, index, step):
        return (
            index + 1,
            step["name"],
            "MoveJ" if step["move"] == "movej" else "MoveL",
            int(step["speed"]),
            (
                ""
                if float(step["wait_s"]) == 0
                else f"{float(step['wait_s']):g}"
            ),
        )

    def _refresh_global_sequence_table(self, select_index=None):
        if not hasattr(self, "global_sequence_table"):
            return

        self._destroy_global_cell_editor(commit=True)

        for iid in self.global_sequence_table.get_children():
            self.global_sequence_table.delete(iid)

        for index, step in enumerate(self.global_point_sequence):
            self.global_sequence_table.insert(
                "",
                "end",
                iid=f"gp_{index}",
                values=self._global_sequence_row_values(index, step),
            )

        if select_index is not None and self.global_point_sequence:
            select_index = max(
                0,
                min(select_index, len(self.global_point_sequence) - 1),
            )
            iid = f"gp_{select_index}"
            self.global_sequence_table.selection_set(iid)
            self.global_sequence_table.focus(iid)
            self.global_sequence_table.see(iid)

    def _selected_global_sequence_index(self):
        selection = self.global_sequence_table.selection()
        if not selection:
            return None

        match = re.match(r"gp_(\d+)$", selection[0])
        return int(match.group(1)) if match else None

    def _remove_global_sequence_step(self):
        index = self._selected_global_sequence_index()
        if index is None:
            return

        self._destroy_global_cell_editor(commit=True)
        del self.global_point_sequence[index]

        self._refresh_global_sequence_table(
            select_index=(
                min(index, len(self.global_point_sequence) - 1)
                if self.global_point_sequence
                else None
            )
        )

    def _move_global_sequence_up(self):
        index = self._selected_global_sequence_index()
        if index is None or index == 0:
            return

        self._destroy_global_cell_editor(commit=True)
        self.global_point_sequence[index - 1], self.global_point_sequence[index] = (
            self.global_point_sequence[index],
            self.global_point_sequence[index - 1],
        )
        self._refresh_global_sequence_table(select_index=index - 1)

    def _move_global_sequence_down(self):
        index = self._selected_global_sequence_index()
        if (
            index is None
            or index >= len(self.global_point_sequence) - 1
        ):
            return

        self._destroy_global_cell_editor(commit=True)
        self.global_point_sequence[index + 1], self.global_point_sequence[index] = (
            self.global_point_sequence[index],
            self.global_point_sequence[index + 1],
        )
        self._refresh_global_sequence_table(select_index=index + 1)

    def _clear_global_sequence(self):
        self._destroy_global_cell_editor(commit=False)
        self.global_point_sequence.clear()
        self._refresh_global_sequence_table()

    @staticmethod
    def _global_column_name(column_id):
        return {
            "#1": "order",
            "#2": "point",
            "#3": "move",
            "#4": "speed",
            "#5": "wait",
        }.get(column_id)

    def _begin_global_cell_edit(self, event):
        self._destroy_global_cell_editor(commit=True)

        if (
            self.global_sequence_table.identify_region(
                event.x,
                event.y,
            )
            != "cell"
        ):
            return

        row_id = self.global_sequence_table.identify_row(event.y)
        column_id = self.global_sequence_table.identify_column(event.x)
        field = self._global_column_name(column_id)

        if not row_id or field not in {"move", "speed", "wait"}:
            return

        match = re.match(r"gp_(\d+)$", row_id)
        if not match:
            return

        row = int(match.group(1))
        bbox = self.global_sequence_table.bbox(row_id, column_id)
        if not bbox:
            return

        x, y, width, height = bbox
        step = self.global_point_sequence[row]

        if field == "move":
            value = "MoveJ" if step["move"] == "movej" else "MoveL"
        elif field == "speed":
            value = str(step["speed"])
        else:
            value = (
                ""
                if float(step["wait_s"]) == 0
                else f"{float(step['wait_s']):g}"
            )

        editor = tk.Entry(
            self.global_sequence_table,
            bg="#102B42",
            fg="#FFFFFF",
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="solid",
            bd=1,
            highlightthickness=1,
            highlightbackground=CYAN,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 9, "bold"),
        )
        editor.insert(0, value)
        editor.select_range(0, "end")
        editor.place(x=x, y=y, width=width, height=height)
        editor.focus_set()

        self._global_cell_editor = {
            "widget": editor,
            "row": row,
            "field": field,
        }

        editor.bind(
            "<Return>",
            lambda _e: self._destroy_global_cell_editor(commit=True),
        )
        editor.bind(
            "<Escape>",
            lambda _e: self._destroy_global_cell_editor(commit=False),
        )
        editor.bind(
            "<FocusOut>",
            lambda _e: self._destroy_global_cell_editor(commit=True),
        )

    def _destroy_global_cell_editor(self, commit):
        info = self._global_cell_editor
        if info is None:
            return True

        self._global_cell_editor = None
        widget = info["widget"]

        try:
            raw = widget.get().strip()
        except Exception:
            raw = ""

        try:
            widget.destroy()
        except Exception:
            pass

        if not commit:
            return True

        row = info["row"]
        field = info["field"]

        if not (0 <= row < len(self.global_point_sequence)):
            return True

        step = self.global_point_sequence[row]

        try:
            if field == "move":
                value = raw.lower().replace(" ", "")
                if value not in ("movej", "movel"):
                    raise ValueError("Motion must be MoveJ or MoveL.")
                step["move"] = value

            elif field == "speed":
                speed = int(float(raw))
                if not 1 <= speed <= 100:
                    raise ValueError("Speed must be 1-100.")
                step["speed"] = speed

            elif field == "wait":
                wait_s = float(raw or "0")
                if not 0 <= wait_s <= 3600:
                    raise ValueError("Wait must be 0-3600 seconds.")
                step["wait_s"] = wait_s

        except Exception as exc:
            messagebox.showerror(
                "Global Point Sequence",
                str(exc),
                parent=self.winfo_toplevel(),
            )
            self._refresh_global_sequence_table(select_index=row)
            return False

        iid = f"gp_{row}"
        if self.global_sequence_table.exists(iid):
            self.global_sequence_table.item(
                iid,
                values=self._global_sequence_row_values(row, step),
            )
            self.global_sequence_table.selection_set(iid)
            self.global_sequence_table.focus(iid)

        return True

    def _run_global_point_sequence(self):
        if self.has_active_command():
            messagebox.showwarning(
                "Arm busy",
                f"{self.side_name.upper()} arm already has an active command.",
                parent=self.winfo_toplevel(),
            )
            return

        if not self.global_point_sequence:
            messagebox.showinfo(
                "Global Point Sequence",
                "Add at least one global point to the sequence.",
                parent=self.winfo_toplevel(),
            )
            return

        if not self._destroy_global_cell_editor(commit=True):
            return

        missing = [
            step["name"]
            for step in self.global_point_sequence
            if self.global_points
            and step["name"] not in self.global_points
        ]
        if missing:
            messagebox.showerror(
                "Global Point Sequence",
                "These points are not in the currently loaded controller list:\n\n"
                + "\n".join(missing),
                parent=self.winfo_toplevel(),
            )
            return

        env = self._validate_environment()
        if not env:
            return

        preview = "\n".join(
            f"{index:02d}. {step['name']} // "
            f"{step['move'].upper()} // {step['speed']}% // "
            f"wait {step['wait_s']:g}s"
            for index, step in enumerate(
                self.global_point_sequence[:12],
                1,
            )
        )
        if len(self.global_point_sequence) > 12:
            preview += (
                f"\n... and "
                f"{len(self.global_point_sequence) - 12} more"
            )

        ok = messagebox.askyesno(
            "Confirm Global Point sequence",
            f"{self.side_name.upper()} ARM // {self.ip_address}\n\n"
            f"{preview}\n\n"
            "For every row:\n"
            "1. Move to the Global Point using MoveJ or MoveL\n"
            "2. Wait for the entered number of seconds\n"
            "3. Continue to the next row\n\n"
            "THE ROBOT CAN MOVE THROUGH MULTIPLE POSITIONS.\n"
            "Confirm the complete path is clear and the hardware E-stop is accessible.",
            icon="warning",
            parent=self.winfo_toplevel(),
        )
        if not ok:
            return

        payload = {
            "format": "realman_global_point_sequence",
            "version": 1,
            "steps": [
                dict(step)
                for step in self.global_point_sequence
            ],
        }

        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                suffix=".json",
                prefix=f"realman_{self.side_name}_global_points_",
                delete=False,
                encoding="utf-8",
            ) as tf:
                json.dump(payload, tf, indent=2)
                sequence_file = tf.name
        except Exception as exc:
            messagebox.showerror(
                "Global Point Sequence",
                f"Could not create sequence file:\n\n{exc}",
                parent=self.winfo_toplevel(),
            )
            return

        rm_upload, sdk = env
        cmd = self._base_command(rm_upload, sdk) + [
            "--global-sequence-file",
            sequence_file,
            "--side",
            self.side_name,
            "--yes",
        ]

        self._start_process(
            cmd,
            (
                f"GLOBAL POINT SEQUENCE // "
                f"{len(self.global_point_sequence)} point(s)"
            ),
            motion=True,
        )

    def _check_arm_mode_ui(self):
        if self.has_active_command():
            messagebox.showwarning("Arm busy", f"{self.side_name.upper()} arm already has an active command.", parent=self.winfo_toplevel())
            return
        env = self._validate_environment()
        if not env:
            return
        rm_upload, sdk = env
        cmd = self._base_command(rm_upload, sdk) + ["--get-arm-mode", "--side", self.side_name, "--yes"]
        self._start_process(cmd, "MODE CHECK // Reading controller run mode", motion=False)

    def _set_arm_mode_ui(self, requested):
        if self.has_active_command():
            messagebox.showwarning("Arm busy", f"{self.side_name.upper()} arm already has an active command.", parent=self.winfo_toplevel())
            return
        requested = str(requested).strip().lower()
        if requested not in ("simulation", "real"):
            return
        if requested == "real":
            ok = messagebox.askyesno(
                "Switch to REAL mode",
                f"{self.side_name.upper()} ARM // {self.ip_address}\n\nSwitch this controller to REAL mode?\n\nFuture robot motion commands can move the physical robotic arm.\nKeep the work area clear and the hardware E-stop accessible.",
                icon="warning", parent=self.winfo_toplevel())
            if not ok:
                return
        env = self._validate_environment()
        if not env:
            return
        rm_upload, sdk = env
        cmd = self._base_command(rm_upload, sdk) + ["--set-arm-mode", requested, "--side", self.side_name, "--yes"]
        self._start_process(cmd, f"MODE SWITCH // Setting {requested.upper()}", motion=False)

    def _browse_arm_track(self):
        downloads = Path.home() / "Downloads"
        initial = downloads if downloads.is_dir() else self.default_track_dir

        current = str(self.track_var.get()).strip()
        if current:
            try:
                p = Path(current).expanduser()
                if p.is_file():
                    initial = p.parent
                elif p.is_dir():
                    initial = p
            except Exception:
                pass

        dialog = TrajectoryFileDialog(
            self.winfo_toplevel(),
            initialdir=initial,
        )
        path = dialog.show()
        if path:
            self.track_var.set(path)

    def _default_drag_save_path(self):
        from datetime import datetime

        folder = Path.home() / "Downloads" / "RealMan_DragTeach"
        try:
            folder.mkdir(parents=True, exist_ok=True)
        except OSError:
            folder = Path.home()

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return folder / f"{self.side_name}_drag_{stamp}.txt"

    def _browse_drag_save(self):
        initial = self.drag_save_var.get().strip()
        if initial:
            p = Path(initial).expanduser()
            initial_dir = p.parent if p.parent.exists() else Path.home()
            initial_name = p.name or f"{self.side_name}_drag.txt"
        else:
            default = self._default_drag_save_path()
            initial_dir = default.parent
            initial_name = default.name

        path = filedialog.asksaveasfilename(
            title=f"Save {self.side_name.upper()} Drag Teach trajectory",
            initialdir=str(initial_dir),
            initialfile=initial_name,
            defaultextension=".txt",
            filetypes=[
                ("Trajectory text", "*.txt"),
                ("All files", "*.*"),
            ],
            parent=self.winfo_toplevel(),
        )
        if path:
            self.drag_save_var.set(path)

    def _drag_teach_toggle(self):
        # Same button STARTS and STOPS the drag session.
        if self.drag_teach_active:
            p = self.process
            if not p or p.poll() is not None:
                self.drag_teach_active = False
                self.drag_toggle_btn.configure(
                    text="✋ START DRAG + RECORD"
                )
                self.status_var.set("DRAG TEACH // Session already stopped")
                return

            self.status_var.set(
                "DRAG TEACH STOP REQUESTED // Stopping recording"
            )
            self._append_log(
                "\n[UI] DRAG button pressed again — stopping Drag Teach...\n"
            )
            self.drag_toggle_btn.configure(state="disabled")

            try:
                if os.name == "nt":
                    p.send_signal(signal.CTRL_BREAK_EVENT)
                else:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception as exc:
                self._append_log(
                    f"[UI] Could not stop Drag Teach process: {exc}\n"
                )
                self.status_var.set("FAULT // Drag Teach stop signal failed")
                self.drag_toggle_btn.configure(state="normal")
            return

        if self.has_active_command():
            messagebox.showwarning(
                "Arm busy",
                f"{self.side_name.upper()} arm already has an active command.",
                parent=self.winfo_toplevel(),
            )
            return

        env = self._validate_environment()
        if not env:
            return

        rm_upload, sdk = env

        ok = messagebox.askyesno(
            "Start Drag & Teach",
            f"{self.side_name.upper()} ARM // {self.ip_address}\n\n"
            "The arm will enter Drag Teach mode and become hand-guidable.\n"
            "Trajectory recording will be enabled.\n\n"
            "Keep the work area clear and keep the hardware E-stop accessible.\n\n"
            "START DRAG + RECORD?",
            icon="warning",
            parent=self.winfo_toplevel(),
        )
        if not ok:
            return

        if not self.drag_save_var.get().strip():
            self.drag_save_var.set(str(self._default_drag_save_path()))

        cmd = self._base_command(rm_upload, sdk) + [
            "--drag-session",
            "--side",
            self.side_name,
            "--yes",
        ]

        self.drag_teach_active = True
        self.drag_teach_has_recording = False
        self.drag_toggle_btn.configure(
            text="■ STOP DRAG",
            state="normal",
        )
        self.drag_save_btn.configure(state="disabled")
        self.drag_playback_btn.configure(state="disabled")

        self._start_process(
            cmd,
            "DRAG TEACH STARTING // Waiting for controller",
            motion=True,
        )

        # _start_process calls _set_busy(True), which normally disables buttons.
        # The same Drag button must remain available as the STOP button.
        self.drag_toggle_btn.configure(state="normal")

    def _drag_session_finished(self):
        self.drag_teach_active = False
        self._set_busy(False)

        self.drag_toggle_btn.configure(
            text="✋ START DRAG + RECORD",
            state="normal",
        )

        if self.drag_teach_has_recording:
            self.drag_save_btn.configure(state="normal")
            self.drag_playback_btn.configure(state="normal")
        else:
            self.drag_save_btn.configure(state="disabled")
            self.drag_playback_btn.configure(state="disabled")


    def _drag_save(self):
        if not self.drag_teach_has_recording:
            messagebox.showinfo(
                "Drag Teach",
                "Record and stop a Drag Teach trajectory first.",
                parent=self.winfo_toplevel(),
            )
            return

        env = self._validate_environment()
        if not env:
            return

        rm_upload, sdk = env

        path = self.drag_save_var.get().strip()
        if not path:
            self._browse_drag_save()
            path = self.drag_save_var.get().strip()
        if not path:
            return

        cmd = self._base_command(rm_upload, sdk) + [
            "--drag-save",
            path,
            "--side",
            self.side_name,
            "--yes",
        ]

        self._start_process(
            cmd,
            "DRAG SAVE // Saving recorded trajectory",
            motion=False,
        )

    def _drag_playback(self):
        if not self.drag_teach_has_recording:
            messagebox.showinfo(
                "Drag Teach",
                "Record and stop a Drag Teach trajectory first.",
                parent=self.winfo_toplevel(),
            )
            return

        env = self._validate_environment()
        if not env:
            return

        rm_upload, sdk = env

        ok = messagebox.askyesno(
            "Playback Drag Teach trajectory",
            f"{self.side_name.upper()} ARM // {self.ip_address}\n\n"
            "The robot will first MOVE to the recorded Drag Teach start position,\n"
            "then replay the complete recorded trajectory.\n\n"
            "The arm will move. Confirm the work area is clear and the "
            "hardware E-stop is accessible.\n\n"
            "MOVE TO START + PLAYBACK?",
            icon="warning",
            parent=self.winfo_toplevel(),
        )
        if not ok:
            return

        cmd = self._base_command(rm_upload, sdk) + [
            "--drag-playback",
            "--side",
            self.side_name,
            "--yes",
        ]

        self._start_process(
            cmd,
            "DRAG PLAYBACK // Moving to recorded start position",
            motion=True,
        )

    def _manual_hand_channel_slider_changed(self, index, value=None):
        try:
            raw = int(round(float(
                self.manual_hand_vars[index].get()
                if value is None
                else value
            )))
        except Exception:
            raw = 0

        raw = max(0, min(1000, raw))
        self.manual_hand_vars[index].set(raw)
        self.manual_hand_entry_vars[index].set(str(raw))

    def _manual_hand_channel_entry_changed(self, index):
        try:
            raw = int(round(float(
                self.manual_hand_entry_vars[index].get().strip()
            )))
        except Exception:
            raw = int(self.manual_hand_vars[index].get())

        raw = max(0, min(1000, raw))
        self.manual_hand_vars[index].set(raw)
        self.manual_hand_entry_vars[index].set(str(raw))

    def _manual_hand_values(self):
        values = []
        for index in range(6):
            self._manual_hand_channel_entry_changed(index)
            values.append(int(self.manual_hand_vars[index].get()))
        return values


    def _manual_pole_slider_changed(self, value=None):
        try:
            mm = int(round(float(
                self.manual_pole_var.get() if value is None else value
            )))
        except Exception:
            mm = 0

        mm = max(0, min(315, mm))
        self.manual_pole_var.set(mm)

        # Keep numeric entry synchronized with slider.
        if hasattr(self, "manual_pole_entry_var"):
            self.manual_pole_entry_var.set(str(mm))

        self.manual_pole_text_var.set(f"{mm} mm")


    def _manual_pole_entry_changed(self):
        try:
            mm = float(self.manual_pole_entry_var.get().strip())
        except Exception:
            mm = float(self.manual_pole_var.get())

        mm = max(0.0, min(315.0, mm))
        mm_i = int(round(mm))

        self.manual_pole_var.set(mm_i)
        self.manual_pole_entry_var.set(str(mm_i))
        self.manual_pole_text_var.set(f"{mm_i} mm")


    def _manual_hand_apply_preset(self, values):
        """
        Set all six hand channels to a shortcut preset and immediately apply.
        Order: Little, Ring, Middle, Index, Thumb Bend, Thumb Rotate.
        """
        if len(values) != 6:
            return

        for index, value in enumerate(values):
            raw = max(0, min(1000, int(value)))
            self.manual_hand_vars[index].set(raw)
            self.manual_hand_entry_vars[index].set(str(raw))

        self._manual_hand_apply()

    def _manual_hand_apply(self):
        env = self._validate_environment()
        if not env:
            return

        rm_upload, sdk = env
        uploader_dir = Path(rm_upload).resolve().parent
        if not (uploader_dir / "hand.py").is_file():
            messagebox.showerror(
                "hand.py missing",
                f"Manual Hand control needs hand.py beside:\n{rm_upload}\n\n"
                f"Copy hand.py into:\n{uploader_dir}",
            )
            return

        values = self._manual_hand_values()

        cmd = self._base_command(rm_upload, sdk) + [
            "--manual-hand-raw",
            *[str(v) for v in values],
            "--side",
            self.side_var.get(),
            "--yes",
        ]

        # APPLY itself is the explicit motion command; no extra popup.
        target_text = " / ".join(str(v) for v in values)
        self._start_process(
            cmd,
            f"MANUAL HAND // 6-channel target {target_text}",
            motion=True,
        )


    def _manual_pole_apply(self):
        env = self._validate_environment()
        if not env:
            return

        rm_upload, sdk = env
        uploader_dir = Path(rm_upload).resolve().parent
        if not (uploader_dir / "pole.py").is_file():
            messagebox.showerror(
                "pole.py missing",
                f"Manual Pole control needs pole.py beside:\n{rm_upload}\n\n"
                f"Copy pole.py into:\n{uploader_dir}",
            )
            return

        # Commit any typed value before sending the command.
        self._manual_pole_entry_changed()
        mm = int(round(float(self.manual_pole_var.get())))
        mm = max(0, min(315, mm))

        cmd = self._base_command(rm_upload, sdk) + [
            "--manual-pole",
            str(mm),
            "--side",
            self.side_var.get(),
            "--yes",
        ]

        # APPLY itself is the user's explicit motion command; no extra popup.
        self._start_process(
            cmd,
            f"MANUAL POLE // Moving to {mm} mm",
            motion=True,
        )


    def _validate_environment(self):
        rm_upload = Path(self.rm_upload_var.get().strip()).expanduser()
        if not rm_upload.is_file():
            messagebox.showerror("Missing file", "Please select a valid rm_upload.py file.")
            return None

        sdk = Path(self.sdk_var.get().strip()).expanduser()
        if not sdk.is_dir():
            messagebox.showerror("Missing SDK", "Please select a valid RealMan SDK directory.")
            return None

        return rm_upload, sdk


    def _validate_common(self):
        env = self._validate_environment()
        if not env:
            return None

        rm_upload, sdk = env

        try:
            slot = int(self.slot_var.get())
            if not 1 <= slot <= 100:
                raise ValueError
        except ValueError:
            messagebox.showerror("Upload slot", "Upload slot must be from 1 to 100.")
            return None

        return rm_upload, sdk, slot


    def _validate_track(self):
        path = Path(self.track_var.get().strip()).expanduser()
        if not path.is_file():
            messagebox.showerror(
                "Trajectory file",
                "Please select a valid trajectory .txt file.",
            )
            return None
        return path


    def _base_command(self, rm_upload, sdk):
        return [
            sys.executable,
            "-u",
            str(rm_upload),
            "--sdk",
            str(sdk),
        ]


    def _upload_only(self):
        common = self._validate_common()
        if not common:
            return
        track = self._validate_track()
        if not track:
            return

        rm_upload, sdk, slot = common

        ok = messagebox.askyesno(
            "Confirm upload",
            f"Upload:\n{track.name}\n\n"
            f"to program slot {slot} on the {self.side_var.get().upper()} arm?\n\n"
            "If the slot already contains a program, it will be replaced.",
        )
        if not ok:
            return

        cmd = self._base_command(rm_upload, sdk) + [
            str(track),
            str(slot),
            "--side",
            self.side_var.get(),
            "--yes",
        ]
        self._start_process(cmd, "UPLOAD // Sending and verifying trajectory...", motion=False)


    def _upload_and_run(self):
        common = self._validate_common()
        if not common:
            return
        track = self._validate_track()
        if not track:
            return

        rm_upload, sdk, slot = common
        side = self.side_var.get()

        ok = messagebox.askyesno(
            "Confirm robot movement",
            f"Trajectory: {track.name}\n"
            f"Program slot: {slot}\n"
            f"Arm: {side.upper()}\n\n"
            "This will UPLOAD, VERIFY, and then MOVE THE ROBOT.\n\n"
            "Confirm the work area is clear, the correct arm is selected, "
            "and the hardware E-stop is accessible.\n\n"
            "Start?",
            icon="warning",
        )
        if not ok:
            return

        cmd = self._base_command(rm_upload, sdk) + [
            str(track),
            str(slot),
            "--side",
            side,
            "--run",
            "--yes",
        ]
        self._start_process(cmd, "RUN QUEUED // Uploading and verifying first...", motion=True)


    def _multi_upload_setup(self):
        """Choose multiple files, assign one Program ID to each, then upload sequentially."""
        if self.process and self.process.poll() is None:
            messagebox.showwarning("Busy", "A controller command is already running.")
            return

        env = self._validate_environment()
        if not env:
            return

        downloads = Path.home() / "Downloads"
        initial = downloads if downloads.is_dir() else self.default_track_dir

        picker = TrajectoryFileDialog(
            self,
            initialdir=initial,
            multiple=True,
        )
        selected = picker.show()
        if not selected:
            return

        # Preserve the file-browser order (latest first by default).
        try:
            start_slot = int(self.slot_var.get())
        except Exception:
            start_slot = 1

        setup = BatchUploadDialog(
            self,
            selected,
            start_slot=start_slot,
        )
        jobs = setup.show()
        if not jobs:
            return

        rm_upload, sdk = env
        self._start_batch_upload(rm_upload, sdk, jobs)


    def _start_batch_upload(self, rm_upload, sdk, jobs):
        if self.process and self.process.poll() is None:
            messagebox.showwarning("Busy", "A controller command is already running.")
            return

        self.batch_active = True
        self.motion_capable_command = False
        self._clear_log()
        self._reset_runtime()
        self.status_var.set(f"BATCH READY // {len(jobs)} files queued")
        self._append_log(
            f"BATCH UPLOAD // {len(jobs)} trajectory file(s) queued\n"
            f"ARM // {self.side_var.get().upper()}\n\n"
        )
        self._set_busy(True)

        self.worker = threading.Thread(
            target=self._batch_upload_worker,
            args=(rm_upload, sdk, list(jobs)),
            daemon=True,
        )
        self.worker.start()


    def _run_streamed_subprocess(self, cmd):
        """Run one rm_upload.py command, stream output to the log, return (rc, lines)."""
        kwargs = dict(
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kwargs["start_new_session"] = True

        proc = subprocess.Popen(cmd, **kwargs)
        self.process = proc
        lines = []

        try:
            for line in proc.stdout:
                lines.append(line)
                self.after(0, self._append_log, line)
            rc = proc.wait()
        finally:
            self.process = None

        return rc, lines


    @staticmethod
    def _parse_program_list_lines(lines):
        """
        Parse runner --list output into {slot: program_name}.

        v41 Multi Upload bug fixed:
          1. This helper is called through self, so it is explicitly static.
          2. The previous raw regex was over-escaped and did not match the
             normal "slot  25 ..." output from the runner.
        """
        held = {}

        for line in lines:
            clean = line.strip()

            match = re.match(
                r"^slot\s+(\d+)\s+[0-9.]+\s+KB\s+speed\s+\S+\s*(.*)$",
                clean,
                re.IGNORECASE,
            )

            if match:
                held[
                    int(match.group(1))
                ] = match.group(2).strip()

        return held


    def _ask_batch_confirmation(self, index, total, path, slot, existing_name):
        """
        Ask on the Tk/UI thread and return:
          True  -> upload this file
          False -> skip this file
          None  -> cancel remaining batch
        """
        done = threading.Event()
        answer = {"value": None}

        def ask():
            if existing_name is not None:
                existing_display = existing_name or "(unnamed program)"
                message = (
                    f"File {index} of {total}\\n\\n"
                    f"Trajectory:\\n{path.name}\\n\\n"
                    f"Target Program ID: {slot}\\n\\n"
                    f"CONTROLLER SLOT {slot} IS ALREADY IN USE:\\n"
                    f"{existing_display}\\n\\n"
                    "YES = replace it and upload this file\\n"
                    "NO = skip this file\\n"
                    "CANCEL = stop the remaining batch"
                )
                title = f"Replace Program ID {slot}?"
                icon = "warning"
            else:
                message = (
                    f"File {index} of {total}\\n\\n"
                    f"Trajectory:\\n{path.name}\\n\\n"
                    f"Target Program ID: {slot}\\n"
                    "Controller slot is EMPTY.\\n\\n"
                    "YES = upload this file\\n"
                    "NO = skip this file\\n"
                    "CANCEL = stop the remaining batch"
                )
                title = f"Upload to Program ID {slot}?"
                icon = "question"

            answer["value"] = messagebox.askyesnocancel(
                title,
                message,
                icon=icon,
                parent=self,
            )
            done.set()

        self.after(0, ask)
        done.wait()
        return answer["value"]


    def _batch_upload_worker(self, rm_upload, sdk, jobs):
        total = len(jobs)
        uploaded = 0
        skipped = 0
        failed = 0
        cancelled = False
        side = self.side_var.get()

        try:
            for index, (path, slot) in enumerate(jobs, 1):
                path = Path(path)

                self.after(
                    0,
                    self.status_var.set,
                    f"BATCH CHECK // {index}/{total} // Slot {slot}",
                )
                self.after(
                    0,
                    self._append_log,
                    (
                        "\\n"
                        + "=" * 72
                        + f"\\nBATCH ITEM {index}/{total}\\n"
                        + f"FILE // {path.name}\\n"
                        + f"PROGRAM ID // {slot}\\n"
                        + "=" * 72
                        + "\\n"
                    ),
                )

                # Read the controller immediately before every upload so the
                # replacement warning reflects its current contents.
                list_cmd = self._base_command(rm_upload, sdk) + [
                    "--list",
                    "--side",
                    side,
                ]
                rc_list, list_lines = self._run_streamed_subprocess(list_cmd)

                if rc_list != 0:
                    failed += 1
                    self.after(
                        0,
                        self._append_log,
                        f"\\n[BATCH] Could not read controller slots before {path.name}; skipping.\\n",
                    )
                    continue

                held = self._parse_program_list_lines(list_lines)

                self.after(
                    0,
                    self._append_log,
                    (
                        f"[BATCH] Controller list OK // "
                        f"{len(held)} stored program(s) found.\n"
                    ),
                )

                # Update the launcher dropdown with this fresh controller list.
                def update_program_cache(snapshot=held):
                    self.controller_programs.clear()
                    self.controller_programs.update(snapshot)
                    self._refresh_program_combo()

                self.after(0, update_program_cache)

                existing = held.get(slot) if slot in held else None
                decision = self._ask_batch_confirmation(
                    index,
                    total,
                    path,
                    slot,
                    existing,
                )

                if decision is None:
                    cancelled = True
                    self.after(
                        0,
                        self._append_log,
                        "\\n[BATCH] User cancelled the remaining batch.\\n",
                    )
                    break

                if decision is False:
                    skipped += 1
                    self.after(
                        0,
                        self._append_log,
                        f"\\n[BATCH] SKIPPED // {path.name} -> slot {slot}\\n",
                    )
                    continue

                self.after(
                    0,
                    self.status_var.set,
                    f"BATCH UPLOAD // {index}/{total} // {path.name} -> {slot}",
                )

                # --yes suppresses rm_upload.py's own terminal Y prompt because
                # the UI has already confirmed this exact file/slot above.
                upload_cmd = self._base_command(rm_upload, sdk) + [
                    str(path),
                    str(slot),
                    "--side",
                    side,
                    "--yes",
                ]

                rc_upload, upload_lines = self._run_streamed_subprocess(upload_cmd)

                verified = (
                    rc_upload == 0
                    and any("VERIFIED:" in line for line in upload_lines)
                )

                if verified:
                    uploaded += 1
                    self.after(
                        0,
                        self._append_log,
                        f"\\n[BATCH] OK // {path.name} uploaded and verified in slot {slot}.\\n",
                    )
                else:
                    failed += 1
                    self.after(
                        0,
                        self._append_log,
                        f"\\n[BATCH] FAILED // {path.name} -> slot {slot}.\\n",
                    )

            # Final controller list so the program launcher is current.
            final_cmd = self._base_command(rm_upload, sdk) + [
                "--list",
                "--side",
                side,
            ]
            rc_final, final_lines = self._run_streamed_subprocess(final_cmd)
            if rc_final == 0:
                final_held = self._parse_program_list_lines(final_lines)

                def set_final_cache(snapshot=final_held):
                    self.controller_programs.clear()
                    self.controller_programs.update(snapshot)
                    self._refresh_program_combo()

                self.after(0, set_final_cache)

            summary = (
                f"BATCH RESULT // Uploaded {uploaded} // "
                f"Skipped {skipped} // Failed {failed}"
            )
            if cancelled:
                summary += " // Cancelled"

            self.after(0, self.status_var.set, summary)
            self.after(
                0,
                self._append_log,
                (
                    "\\n"
                    + "=" * 72
                    + "\\n"
                    + summary
                    + "\\n"
                    + "=" * 72
                    + "\\n"
                ),
            )

        except Exception as exc:
            self.after(
                0,
                self._append_log,
                f"\\n[BATCH UI ERROR] {exc}\\n",
            )
            self.after(
                0,
                self.status_var.set,
                "FAULT // Batch upload stopped",
            )
        finally:
            self.process = None
            self.batch_active = False
            self.motion_capable_command = False
            self.after(0, self._set_busy, False)


    def _run_upload_slot_only(self):
        """Run the controller program in the current Upload slot without uploading."""
        common = self._validate_common()
        if not common:
            return

        rm_upload, sdk, slot = common
        side = self.side_var.get()

        known_name = self.controller_programs.get(slot, "")
        program_text = f"{slot}"
        if known_name:
            program_text += f"  ({known_name})"

        ok = messagebox.askyesno(
            "Confirm robot movement",
            f"Run controller program slot: {program_text}\n"
            f"Arm: {side.upper()}\n\n"
            "NO FILE WILL BE UPLOADED.\n"
            "The existing program already stored in this slot will run.\n\n"
            "THE ROBOT WILL MOVE.\n\n"
            "Confirm the work area is clear, the correct arm is selected, "
            "and the hardware E-stop is accessible.\n\n"
            "Start?",
            icon="warning",
        )
        if not ok:
            return

        cmd = self._base_command(rm_upload, sdk) + [
            "--run",
            str(slot),
            "--side",
            side,
            "--yes",
        ]
        self._start_process(
            cmd,
            f"RUN ONLY // Launching controller slot {program_text}",
            motion=True,
        )


    @staticmethod
    def _is_drag_program_name(name):
        return "DT_" in str(name or "").upper()

    def _program_combo_values(self):
        values = []
        for sid, name in sorted(self.controller_programs.items()):
            display_name = name if name else "(unnamed)"
            if self._is_drag_program_name(name):
                display_name = f"[DRAG TEACH] {display_name}"
            values.append(f"{sid:03d}  |  {display_name}")
        return values


    def _refresh_program_combo(self):
        if not hasattr(self, "program_combo"):
            return
        self.program_combo.configure(values=self._program_combo_values())
        count = len(self.controller_programs)
        drag_count = sum(
            1
            for name in self.controller_programs.values()
            if self._is_drag_program_name(name)
        )
        self.program_count_var.set(
            (
                f"{count} stored program{'s' if count != 1 else ''} loaded "
                f"// {drag_count} Drag Teach"
            )
            if count
            else "No stored programs found"
        )


    def _remember_program(self, slot, name):
        self.controller_programs[int(slot)] = (name or "").strip()
        self._refresh_program_combo()


    def _resolve_stored_program(self):
        raw = self.stored_program_var.get().strip()
        if not raw:
            messagebox.showinfo(
                "Stored program",
                "Enter a controller program ID or program name.",
            )
            return None

        # A plain numeric ID, or a dropdown value beginning with an ID.
        m = re.match(r"^\s*(\d{1,3})(?:\s*(?:\||—|-).*)?$", raw)
        if m:
            slot = int(m.group(1))
            if not 1 <= slot <= 100:
                messagebox.showerror(
                    "Stored program",
                    "Controller program ID must be from 1 to 100.",
                )
                return None
            return slot, self.controller_programs.get(slot, "")

        if not self.controller_programs:
            messagebox.showinfo(
                "Program names not loaded",
                "To run by program name, click 'REFRESH PROGRAM LIST' first.\n\n"
                "You can still run immediately by entering a numeric program ID.",
            )
            return None

        wanted = raw.casefold()

        # Exact program-name match.
        exact = [
            (sid, name)
            for sid, name in self.controller_programs.items()
            if name and name.casefold() == wanted
        ]
        if len(exact) == 1:
            return exact[0]

        # Also permit a unique partial name for convenience.
        partial = [
            (sid, name)
            for sid, name in self.controller_programs.items()
            if name and wanted in name.casefold()
        ]
        if len(partial) == 1:
            return partial[0]

        if len(partial) > 1:
            sample = "\n".join(
                f"  {sid}: {name}" for sid, name in sorted(partial)[:8]
            )
            messagebox.showinfo(
                "Multiple matching programs",
                "That name matches more than one controller program.\n\n"
                f"{sample}\n\n"
                "Enter the exact name or use the program ID.",
            )
        else:
            messagebox.showerror(
                "Program not found",
                f"No loaded controller program matches:\n{raw}\n\n"
                "Click 'REFRESH PROGRAM LIST' and try again.",
            )
        return None


    def _sequence_folder(self):
        folder = Path.home() / "Downloads" / "RealMan_Sequences"
        try:
            folder.mkdir(parents=True, exist_ok=True)
            return folder
        except OSError:
            return Path.home()


    def _normalize_hand_for_main(raw):
        s = str(raw or "").strip()
        if not s:
            return None

        if "," in s:
            parts = [p.strip() for p in s.split(",")]
            if len(parts) != 6:
                raise ValueError("Hand needs exactly 6 comma-separated values.")
            values = [int(p) for p in parts]
            if any(v < 0 or v > 1000 for v in values):
                raise ValueError("Every Hand channel must be 0–1000.")
            return ",".join(str(v) for v in values)

        if s.endswith("%"):
            s = s[:-1].strip()

        pct = float(s)
        if not 0 <= pct <= 100:
            raise ValueError("Hand percentage must be 0–100.")
        return f"{pct:g}%"


    def _normalize_pole_for_main(raw):
        if raw in (None, ""):
            return None
        mm = float(raw)
        if not 0 <= mm <= 315:
            raise ValueError("Pole height must be 0–315 mm.")
        return float(mm)


    def _normalize_wait_for_main(raw):
        if raw in (None, ""):
            return 0.0
        seconds = float(raw)
        if not 0 <= seconds <= 3600:
            raise ValueError("Wait must be 0–3600 seconds.")
        return float(seconds)


    def _read_saved_sequence_main(self, path):
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except Exception as exc:
            messagebox.showerror(
                "Load Sequence",
                f"Could not read the sequence file:\n\n{exc}",
            )
            return None

        if not isinstance(data, dict) or data.get("format") != "realman_multi_run_sequence":
            messagebox.showerror(
                "Load Sequence",
                "This file is not a valid RealMan Multi Run sequence.",
            )
            return None

        raw = data.get("programs")
        if not isinstance(raw, list) or not raw:
            messagebox.showerror(
                "Load Sequence",
                "The saved sequence contains no programs.",
            )
            return None

        steps = []
        try:
            for item in raw:
                if not isinstance(item, dict):
                    continue

                sid = int(item.get("id"))
                if not 1 <= sid <= 100:
                    continue

                steps.append({
                    "id": sid,
                    "name": str(item.get("name") or ""),
                    "hand": self._normalize_hand_for_main(item.get("hand")),
                    "pole_mm": self._normalize_pole_for_main(item.get("pole_mm")),
                    "wait_s": self._normalize_wait_for_main(
                        item.get("wait_s", item.get("wait"))
                    ),
                })
        except Exception as exc:
            messagebox.showerror(
                "Load Sequence",
                f"Invalid Hand / Pole / Wait value:\n\n{exc}",
            )
            return None

        return steps or None


    def _validate_saved_sequence_main(self, steps):
        missing = []
        renamed = []
        validated = []

        for step in steps:
            sid = step["id"]
            saved_name = step.get("name", "")

            if sid not in self.controller_programs:
                missing.append((sid, saved_name))
                continue

            current_name = self.controller_programs.get(sid, "")
            if saved_name and current_name and saved_name != current_name:
                renamed.append((sid, saved_name, current_name))

            updated = dict(step)
            updated["name"] = current_name
            validated.append(updated)

        if missing:
            details = "\n".join(
                f"{sid}: {name or '(unnamed)'}"
                for sid, name in missing[:12]
            )
            messagebox.showerror(
                "Sequence Cannot Run",
                "These Program IDs are not currently on the controller:\n\n"
                f"{details}",
            )
            return None

        if renamed:
            details = "\n\n".join(
                f"Program {sid}\nSaved:   {old or '(unnamed)'}\n"
                f"Current: {new or '(unnamed)'}"
                for sid, old, new in renamed[:10]
            )
            use_current = messagebox.askyesno(
                "Stored Programs Changed",
                "Some Program IDs now contain different program names:\n\n"
                f"{details}\n\n"
                "Use the CURRENT programs with the saved Hand / Pole / Wait values?",
                icon="warning",
            )
            if not use_current:
                return None

        return validated


    def _step_preview(step, controller_programs):
        sid = step["id"]
        name = controller_programs.get(sid, "")
        aux = []

        if step.get("pole_mm") is not None:
            aux.append(f"Pole {float(step['pole_mm']):g} mm")

        if step.get("hand"):
            aux.append(f"Hand {step['hand']}")

        wait_s = float(step.get("wait_s") or 0.0)
        if wait_s > 0:
            aux.append(f"Wait {wait_s:g} s")

        suffix = " | " + ", ".join(aux) if aux else ""
        return f"{sid}" + (f"  {name}" if name else "") + suffix


    def _load_sequence_and_run(self):
        if self.process and self.process.poll() is None:
            messagebox.showwarning("Busy", "A controller command is already running.")
            return

        if self.batch_active or self.multi_run_active:
            messagebox.showwarning("Busy", "Another batch or sequence operation is active.")
            return

        env = self._validate_environment()
        if not env:
            return

        if not self.controller_programs:
            messagebox.showinfo(
                "Program list required",
                "Click 'REFRESH PROGRAM LIST' first.",
            )
            return

        path = filedialog.askopenfilename(
            title="Load saved RealMan sequence",
            initialdir=str(self._sequence_folder()),
            filetypes=[
                ("RealMan sequence", "*.json"),
                ("All files", "*.*"),
            ],
            parent=self,
        )
        if not path:
            return

        steps = self._read_saved_sequence_main(path)
        if not steps:
            return

        sequence = self._validate_saved_sequence_main(steps)
        if not sequence:
            return

        rm_upload, sdk = env
        side = self.side_var.get()

        uploader_dir = Path(rm_upload).resolve().parent

        if any(step.get("hand") for step in sequence):
            if not (uploader_dir / "hand.py").is_file():
                messagebox.showerror(
                    "hand.py missing",
                    f"This sequence uses Hand positions, but hand.py is not beside:\n{rm_upload}\n\n"
                    f"Copy hand.py into:\n{uploader_dir}",
                )
                return

        if any(step.get("pole_mm") is not None for step in sequence):
            if not (uploader_dir / "pole.py").is_file():
                messagebox.showerror(
                    "pole.py missing",
                    f"This sequence uses Pole positions, but pole.py is not beside:\n{rm_upload}\n\n"
                    f"Copy pole.py into:\n{uploader_dir}",
                )
                return

        preview_lines = [
            f"{i:02d}.  {self._step_preview(step, self.controller_programs)}"
            for i, step in enumerate(sequence, 1)
        ]
        preview = "\n".join(preview_lines[:12])
        if len(sequence) > 12:
            preview += f"\n... and {len(sequence) - 12} more"

        ok = messagebox.askyesno(
            "Confirm saved sequence",
            f"Sequence file: {Path(path).name}\n"
            f"Arm: {side.upper()}\n"
            f"Programs: {len(sequence)}\n\n"
            f"{preview}\n\n"
            "Each row executes:\n"
            "POLE → HAND → WAIT → TRAJECTORY\n\n"
            "Blank Hand/Pole fields keep the current position. "
            "Blank/0 Wait means no delay.\n\n"
            "SLOW STOP stops auxiliary motion, stops the arm, "
            "and cancels the remaining sequence.\n\n"
            "Run this saved sequence now?",
            icon="warning",
        )
        if not ok:
            return

        self._start_multi_run(rm_upload, sdk, sequence)


    def _multi_run_setup(self):
        if self.process and self.process.poll() is None:
            messagebox.showwarning("Busy", "A controller command is already running.")
            return

        if self.batch_active or self.multi_run_active:
            messagebox.showwarning("Busy", "Another batch or sequence operation is active.")
            return

        env = self._validate_environment()
        if not env:
            return

        if not self.controller_programs:
            messagebox.showinfo(
                "Program list required",
                "Click 'REFRESH PROGRAM LIST' first, then use MULTI RUN.",
            )
            return

        dialog = MultiRunDialog(self, self.controller_programs)
        sequence = dialog.show()
        if not sequence:
            return

        rm_upload, sdk = env
        side = self.side_var.get()
        uploader_dir = Path(rm_upload).resolve().parent

        if any(step.get("hand") for step in sequence):
            if not (uploader_dir / "hand.py").is_file():
                messagebox.showerror(
                    "hand.py missing",
                    f"This sequence uses Hand positions, but hand.py is not beside:\n{rm_upload}\n\n"
                    f"Copy hand.py into:\n{uploader_dir}",
                )
                return

        if any(step.get("pole_mm") is not None for step in sequence):
            if not (uploader_dir / "pole.py").is_file():
                messagebox.showerror(
                    "pole.py missing",
                    f"This sequence uses Pole positions, but pole.py is not beside:\n{rm_upload}\n\n"
                    f"Copy pole.py into:\n{uploader_dir}",
                )
                return

        preview_lines = [
            f"{i:02d}.  {self._step_preview(step, self.controller_programs)}"
            for i, step in enumerate(sequence, 1)
        ]
        preview = "\n".join(preview_lines[:12])
        if len(sequence) > 12:
            preview += f"\n... and {len(sequence) - 12} more"

        ok = messagebox.askyesno(
            "Confirm multi-run sequence",
            f"Arm: {side.upper()}\nPrograms: {len(sequence)}\n\n"
            f"{preview}\n\n"
            "Every table row executes:\n"
            "1. POLE target (if entered)\n"
            "2. HAND target (if entered)\n"
            "3. WAIT in seconds (if > 0)\n"
            "4. ROBOT TRAJECTORY\n\n"
            "SLOW STOP stops auxiliary motion, stops the arm, "
            "and cancels all remaining programs.\n\n"
            "Confirm the work area is clear and the hardware E-stop is accessible.\n\n"
            "Start the complete sequence?",
            icon="warning",
        )
        if not ok:
            return

        self._start_multi_run(rm_upload, sdk, sequence)


    def _start_multi_run(self, rm_upload, sdk, sequence):
        import time

        self.multi_run_active = True
        self.sequence_cancel_requested = False
        self.motion_capable_command = True
        self.sequence_started_at = time.monotonic()

        self._clear_log()
        self._reset_runtime()

        self.status_var.set(
            f"FAST MULTI RUN READY // {len(sequence)} programs queued"
        )

        self._append_log(
            f"FAST MULTI RUN // {len(sequence)} program(s) queued\n"
            f"ARM // {self.side_var.get().upper()}\n"
            "MODE // One persistent RealMan controller connection\n"
            "STEP ORDER // Pole → Hand → Wait → Robot trajectory\n\n"
        )

        for i, step in enumerate(sequence, 1):
            self._append_log(
                f"  {i:02d}. Program "
                + self._step_preview(step, self.controller_programs)
                + "\n"
            )

        self._append_log("\n")
        self._set_busy(True)

        self.worker = threading.Thread(
            target=self._multi_run_worker,
            args=(rm_upload, sdk, [dict(step) for step in sequence]),
            daemon=True,
        )
        self.worker.start()


    def _multi_run_worker(self, rm_upload, sdk, sequence):
        """Fast Multi Run: Pole -> Hand -> Wait -> Trajectory."""
        import time

        total = len(sequence)
        side = self.side_var.get()
        sequence_t0 = time.monotonic()
        completed = 0
        final_total = None
        temp_path = None

        payload = {
            "format": "realman_multi_run_sequence",
            "version": 3,
            "programs": sequence,
        }

        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                suffix=".json",
                prefix="realman_run_",
                delete=False,
                encoding="utf-8",
            ) as tf:
                json.dump(payload, tf, indent=2)
                temp_path = tf.name

            cmd = self._base_command(rm_upload, sdk) + [
                "--sequence-file",
                temp_path,
                "--side",
                side,
                "--yes",
            ]

            kwargs = dict(
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )

            if os.name == "nt":
                kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
            else:
                kwargs["start_new_session"] = True

            self.after(
                0,
                self._append_log,
                "[FAST MULTI RUN] One persistent controller connection for Pole + Hand + Wait + robot sequence.\n\n",
            )

            proc = subprocess.Popen(cmd, **kwargs)
            self.process = proc

            for line in proc.stdout:
                self.after(0, self._append_log, line)
                clean = line.strip()

                if clean.startswith("AUX POLE ") and "COMPLETE" not in clean:
                    self.after(
                        0,
                        self.status_var.set,
                        "AUX // Moving Pole",
                    )
                    continue

                if clean.startswith("AUX HAND ") and "COMPLETE" not in clean:
                    self.after(
                        0,
                        self.status_var.set,
                        "AUX // Moving Hand",
                    )
                    continue

                if clean.startswith("AUX WAIT ") and "COMPLETE" not in clean:
                    self.after(
                        0,
                        self.status_var.set,
                        "AUX // Waiting before trajectory",
                    )
                    continue

                m = re.match(
                    r"^SEQUENCE PROGRAM\s+(\d+)/(\d+):\s+slot\s+(\d+)\s+\((.*)\)$",
                    clean,
                )
                if m:
                    index = int(m.group(1))
                    total = int(m.group(2))
                    slot = int(m.group(3))
                    name = m.group(4).strip()
                    label = f"{slot}" + (f" ({name})" if name else "")

                    self.after(0, self._reset_runtime)
                    self.after(
                        0,
                        self.status_var.set,
                        f"FAST MULTI RUN // {index}/{total} // Program {label}",
                    )
                    continue

                if "warning-only arm code" in clean.lower():
                    self.after(
                        0,
                        self.status_var.set,
                        "WARNING ONLY // 4105 retained // Continuing sequence",
                    )
                    continue

                if clean.startswith("SEQUENCE RUNNING "):
                    self.after(0, self._start_runtime_timer)
                    continue

                m = re.match(
                    r"^SEQUENCE PROGRAM COMPLETE\s+(\d+)/(\d+):\s+([0-9]+(?:\.[0-9]+)?)\s+s$",
                    clean,
                )
                if m:
                    completed = int(m.group(1))
                    seconds = float(m.group(3))
                    self.after(0, self._stop_runtime_timer, seconds)
                    continue

                m = re.match(
                    r"^SEQUENCE SETTLE\s+(\d+)/(\d+):\s+([0-9]+(?:\.[0-9]+)?)\s+s$",
                    clean,
                )
                if m:
                    self.after(
                        0,
                        self.status_var.set,
                        f"SWITCH // {m.group(1)}/{m.group(2)} complete // "
                        f"settle {float(m.group(3)):.3f} s",
                    )
                    continue

                m = re.match(
                    r"^SEQUENCE COMPLETE:\s+(\d+)/(\d+)\s+programs,\s+([0-9]+(?:\.[0-9]+)?)\s+s$",
                    clean,
                )
                if m:
                    completed = int(m.group(1))
                    total = int(m.group(2))
                    final_total = float(m.group(3))

            rc = proc.wait()
            self.process = None

            if self.run_started_at is not None:
                self.after(0, self._stop_runtime_timer)

            elapsed = (
                final_total
                if final_total is not None
                else time.monotonic() - sequence_t0
            )

            if self.sequence_cancel_requested:
                final_status = (
                    f"FAST MULTI RUN STOPPED // {completed}/{total} completed "
                    f"// Total {elapsed:.1f} s"
                )
            elif rc != 0:
                final_status = (
                    f"FAST MULTI RUN FAILED // {completed}/{total} completed "
                    f"// Total {elapsed:.1f} s"
                )
            else:
                final_status = (
                    f"FAST MULTI RUN COMPLETE // {completed}/{total} completed "
                    f"// Total {elapsed:.1f} s"
                )

            self.after(0, self.status_var.set, final_status)
            self.after(
                0,
                self._append_log,
                "\n" + "=" * 72 + "\n"
                + final_status
                + "\n" + "=" * 72 + "\n",
            )

        except Exception as exc:
            self.after(
                0,
                self._append_log,
                f"\n[FAST MULTI RUN UI ERROR] {exc}\n",
            )
            self.after(
                0,
                self.status_var.set,
                "FAULT // Fast multi-run sequence stopped",
            )

        finally:
            self.process = None

            if self.run_started_at is not None:
                self.after(0, self._stop_runtime_timer)

            if temp_path:
                try:
                    Path(temp_path).unlink(missing_ok=True)
                except Exception:
                    pass

            self.multi_run_active = False
            self.motion_capable_command = False
            self.sequence_started_at = None
            self.after(0, self._set_busy, False)


    def _run_existing(self):
        env = self._validate_environment()
        if not env:
            return

        resolved = self._resolve_stored_program()
        if not resolved:
            return

        rm_upload, sdk = env
        slot, name = resolved
        side = self.side_var.get()

        program_text = f"{slot}"
        if name:
            program_text += f"  ({name})"

        ok = messagebox.askyesno(
            "Confirm robot movement",
            f"Controller program: {program_text}\n"
            f"Arm: {side.upper()}\n\n"
            "THE ROBOT WILL MOVE.\n\n"
            "Confirm the work area is clear, the correct arm is selected, "
            "and the hardware E-stop is accessible.\n\n"
            "Start?",
            icon="warning",
        )
        if not ok:
            return

        cmd = self._base_command(rm_upload, sdk) + [
            "--run",
            str(slot),
            "--side",
            side,
            "--yes",
        ]
        self._start_process(
            cmd,
            f"LAUNCHING // Program {program_text}",
            motion=True,
        )


    def _list_slots(self):
        env = self._validate_environment()
        if not env:
            return

        rm_upload, sdk = env

        # Clear stale names before loading the current controller list.
        self.controller_programs.clear()
        self.program_count_var.set("Reading controller program list...")
        self._refresh_program_combo()

        cmd = self._base_command(rm_upload, sdk) + [
            "--list",
            "--side",
            self.side_var.get(),
        ]
        self._start_process(
            cmd,
            "SYNC // Reading controller program slots...",
            motion=False,
        )


    def _reset_runtime(self):
        if self.runtime_after_id is not None:
            try:
                self.after_cancel(self.runtime_after_id)
            except Exception:
                pass
            self.runtime_after_id = None
        self.run_started_at = None
        self.runtime_var.set("0.0 s")


    def _start_runtime_timer(self):
        if self.run_started_at is not None:
            return
        import time
        self.run_started_at = time.monotonic()
        self._update_runtime_timer()


    def _update_runtime_timer(self):
        if self.run_started_at is None:
            return
        import time
        elapsed = time.monotonic() - self.run_started_at
        self.runtime_var.set(f"{elapsed:.1f} s")
        self.runtime_after_id = self.after(100, self._update_runtime_timer)


    def _stop_runtime_timer(self, final_seconds=None):
        if self.runtime_after_id is not None:
            try:
                self.after_cancel(self.runtime_after_id)
            except Exception:
                pass
            self.runtime_after_id = None

        if final_seconds is not None:
            self.runtime_var.set(f"{final_seconds:.1f} s")
        elif self.run_started_at is not None:
            import time
            elapsed = time.monotonic() - self.run_started_at
            self.runtime_var.set(f"{elapsed:.1f} s")

        self.run_started_at = None


    def _set_busy(self, busy):
        state = "disabled" if busy else "normal"
        self.upload_btn.configure(state=state)
        self.upload_run_btn.configure(state=state)
        self.run_only_btn.configure(state=state)
        self.multi_upload_btn.configure(state=state)
        self.run_btn.configure(state=state)
        self.multi_run_btn.configure(state=state)
        self.load_sequence_btn.configure(state=state)
        self.list_btn.configure(state=state)
        self.manual_hand_apply_btn.configure(state=state)
        self.manual_hand_grip_btn.configure(state=state)
        self.manual_hand_open_btn.configure(state=state)
        for slider in self.manual_hand_sliders:
            slider.configure(state=state)
        for entry in self.manual_hand_entries:
            entry.configure(state=state)
        self.manual_pole_apply_btn.configure(state=state)
        self.sim_mode_btn.configure(state=state)
        self.real_mode_btn.configure(state=state)
        self.check_mode_btn.configure(state=state)

        self.global_refresh_btn.configure(state=state)
        self.global_movej_btn.configure(state=state)
        self.global_movel_btn.configure(state=state)
        self.global_add_btn.configure(state=state)
        self.global_up_btn.configure(state=state)
        self.global_down_btn.configure(state=state)
        self.global_remove_btn.configure(state=state)
        self.global_clear_btn.configure(state=state)
        self.global_sequence_run_btn.configure(state=state)
        self.global_search_entry.configure(state=state)
        self.global_speed_entry.configure(state=state)
        self.global_wait_entry.configure(state=state)

        # During an active Drag Teach session the same button remains available
        # so the user can press it again to stop recording.
        if self.drag_teach_active and busy:
            self.drag_toggle_btn.configure(
                state="normal",
                text="■ STOP DRAG",
            )
        else:
            self.drag_toggle_btn.configure(
                state=state,
                text="✋ START DRAG + RECORD",
            )

        if busy:
            self.drag_save_btn.configure(state="disabled")
            self.drag_playback_btn.configure(state="disabled")
            self.drag_browse_btn.configure(state="disabled")
            self.drag_save_entry.configure(state="disabled")
        else:
            self.drag_browse_btn.configure(state="normal")
            self.drag_save_entry.configure(state="normal")
            if self.drag_teach_has_recording:
                self.drag_save_btn.configure(state="normal")
                self.drag_playback_btn.configure(state="normal")
            else:
                self.drag_save_btn.configure(state="disabled")
                self.drag_playback_btn.configure(state="disabled")

        self.program_combo.configure(state="disabled" if busy else "normal")

        stop_state = (
            "normal"
            if busy and self.motion_capable_command
            else "disabled"
        )
        self.stop_btn.configure(state=stop_state)
        self.quick_stop_btn.configure(state=stop_state)


    def _append_log(self, text):
        # Auto-follow new output only when the user is already at the bottom.
        # If the user scrolls up to read older lines, keep that scroll position.
        try:
            _top, bottom = self.log.yview()
            was_at_bottom = bottom >= 0.999
        except Exception:
            was_at_bottom = True

        self.log.configure(state="normal")
        self.log.insert("end", text)

        if was_at_bottom:
            self.log.see("end")

        self.log.configure(state="disabled")


    def _clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")


    def _start_process_synced(self, cmd, initial_status, start_barrier):
        """
        Prepare this arm exactly like a normal motion command, but wait on a
        shared threading.Barrier immediately before launching the subprocess.

        Used by DUAL RUN so LEFT and RIGHT process launch happens concurrently.
        """
        if self.process and self.process.poll() is None:
            raise RuntimeError(
                f"{self.side_name.upper()} arm already has an active controller command"
            )

        self.last_command_rc = None
        self.motion_capable_command = True
        self._clear_log()
        self._reset_runtime()
        self._append_log(
            "[DUAL RUN] Waiting for both arms at synchronized launch barrier...\n"
        )
        self._append_log(
            "$ " + " ".join(
                f'"{x}"' if " " in x else x
                for x in cmd
            ) + "\n\n"
        )
        self.status_var.set(initial_status)
        self._set_busy(True)

        self.worker = threading.Thread(
            target=self._synced_process_worker,
            args=(cmd, start_barrier),
            daemon=True,
        )
        self.worker.start()

    def _synced_process_worker(self, cmd, start_barrier):
        try:
            # Both LEFT and RIGHT worker threads reach this point before either
            # starts rm_upload. This minimizes software-side launch offset.
            start_barrier.wait(timeout=8.0)
        except Exception as exc:
            self.after(
                0,
                self._append_log,
                f"\n[DUAL RUN] Synchronization failed: {exc}\n",
            )
            self.after(
                0,
                self.status_var.set,
                "FAULT // Dual-run synchronization failed",
            )
            self.motion_capable_command = False
            self.after(0, self._set_busy, False)
            return

        self._process_worker(cmd)


    def _start_process(self, cmd, initial_status, motion=False):
        if self.process and self.process.poll() is None:
            messagebox.showwarning("Busy", "A controller command is already running.")
            return

        self.last_command_rc = None
        self.motion_capable_command = bool(motion)
        self._clear_log()
        self._reset_runtime()
        self._append_log("$ " + " ".join(f'"{x}"' if " " in x else x for x in cmd) + "\n\n")
        self.status_var.set(initial_status)
        self._set_busy(True)

        self.worker = threading.Thread(
            target=self._process_worker,
            args=(cmd,),
            daemon=True,
        )
        self.worker.start()


    def _process_worker(self, cmd):
        is_drag_session = "--drag-session" in cmd
        is_global_sequence = "--global-sequence-file" in cmd
        is_global_single = "--global-point" in cmd
        try:
            kwargs = dict(
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )

            if os.name == "nt":
                kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
            else:
                kwargs["start_new_session"] = True

            self.process = subprocess.Popen(cmd, **kwargs)

            for line in self.process.stdout:
                self.after(0, self._append_log, line)
                clean = line.strip()

                # rm_upload.py --list prints lines like:
                #   slot  25     12.3 KB  speed 0  program_name
                slot_match = re.match(
                    r"^slot\s+(\d+)\s+[0-9.]+\s+KB\s+speed\s+\S+\s*(.*)$",
                    clean,
                    re.IGNORECASE,
                )
                if slot_match:
                    sid = int(slot_match.group(1))
                    pname = slot_match.group(2).strip()
                    self.after(0, self._remember_program, sid, pname)

                if clean.startswith("GLOBAL POINT:"):
                    try:
                        point = json.loads(
                            clean[len("GLOBAL POINT:"):].strip()
                        )
                    except Exception:
                        point = None

                    if point:
                        self.after(
                            0,
                            self._remember_global_point,
                            point,
                        )

                elif clean.startswith("GLOBAL POINT LIST COMPLETE:"):
                    self.after(
                        0,
                        self.status_var.set,
                        (
                            f"GLOBAL POINTS READY // "
                            f"{len(self.global_points)} loaded"
                        ),
                    )

                elif clean.startswith("GLOBAL SEQUENCE START:"):
                    self.after(
                        0,
                        self.status_var.set,
                        "GLOBAL POINT SEQUENCE // Running",
                    )
                    self.after(0, self._start_runtime_timer)

                elif clean.startswith("GLOBAL SEQUENCE STEP "):
                    self.after(
                        0,
                        self.status_var.set,
                        clean.replace(
                            "GLOBAL SEQUENCE STEP",
                            "GLOBAL STEP",
                            1,
                        ),
                    )

                elif clean.startswith("GLOBAL SEQUENCE COMPLETE:"):
                    self.after(
                        0,
                        self.status_var.set,
                        "GLOBAL POINT SEQUENCE COMPLETE",
                    )
                    match_global = re.search(
                        r"//\s*([0-9]+(?:\.[0-9]+)?)\s+s$",
                        clean,
                    )
                    self.after(
                        0,
                        self._stop_runtime_timer,
                        (
                            float(match_global.group(1))
                            if match_global
                            else None
                        ),
                    )

                elif clean.startswith("GLOBAL MOVE START:"):
                    if is_global_single:
                        self.after(0, self._start_runtime_timer)

                    self.after(
                        0,
                        self.status_var.set,
                        clean.replace(
                            "GLOBAL MOVE START:",
                            "GLOBAL MOVE //",
                            1,
                        ),
                    )

                elif clean.startswith("GLOBAL MOVE COMPLETE:"):
                    if is_global_single:
                        self.after(0, self._stop_runtime_timer)

                    self.after(
                        0,
                        self.status_var.set,
                        clean.replace(
                            "GLOBAL MOVE COMPLETE:",
                            "GLOBAL MOVE COMPLETE //",
                            1,
                        ),
                    )

                elif clean.startswith("GLOBAL WAIT:"):
                    self.after(
                        0,
                        self.status_var.set,
                        clean.replace(
                            "GLOBAL WAIT:",
                            "GLOBAL WAIT //",
                            1,
                        ),
                    )

                elif clean.startswith("** GLOBAL"):
                    self.after(
                        0,
                        self.status_var.set,
                        "FAULT // Global Point command failed",
                    )

                elif clean.startswith("ARM RUN MODE:"):
                    if "SIMULATION" in clean:
                        self.after(0, self.arm_mode_var.set, "MODE // SIMULATION")
                        self.after(0, self.status_var.set, "MODE CHECK COMPLETE // SIMULATION")
                    elif "REAL" in clean:
                        self.after(0, self.arm_mode_var.set, "MODE // REAL")
                        self.after(0, self.status_var.set, "MODE CHECK COMPLETE // REAL")
                elif clean.startswith("ARM MODE SET COMPLETE:"):
                    if "SIMULATION" in clean:
                        self.after(0, self.arm_mode_var.set, "MODE // SIMULATION")
                        self.after(0, self.status_var.set, "MODE SWITCH COMPLETE // SIMULATION")
                    elif "REAL" in clean:
                        self.after(0, self.arm_mode_var.set, "MODE // REAL")
                        self.after(0, self.status_var.set, "MODE SWITCH COMPLETE // REAL")
                elif clean.startswith("** ARM MODE"):
                    self.after(0, self.status_var.set, "FAULT // Arm run-mode command failed")
                elif clean.startswith("DRAG TEACH ACTIVE:"):
                    self.after(
                        0,
                        self.status_var.set,
                        "DRAG TEACH ACTIVE // Arm is free // Recording trajectory",
                    )
                elif clean.startswith("DRAG TEACH STOPPED:"):
                    self.drag_teach_has_recording = True
                    self.after(
                        0,
                        self.status_var.set,
                        "DRAG TEACH STOPPED // Recording ready to save/playback",
                    )
                elif clean.startswith("DRAG SAVE COMPLETE:"):
                    self.after(
                        0,
                        self.status_var.set,
                        "DRAG SAVE COMPLETE // Trajectory file saved",
                    )
                elif clean.startswith("DRAG ORIGIN COMPLETE:"):
                    self.after(
                        0,
                        self.status_var.set,
                        "DRAG PLAYBACK // At recorded start position",
                    )
                elif clean.startswith("DRAG PLAYBACK START:"):
                    self.after(
                        0,
                        self.status_var.set,
                        "DRAG PLAYBACK ACTIVE // Robot moving",
                    )
                    self.after(0, self._start_runtime_timer)
                elif clean.startswith("DRAG PLAYBACK COMPLETE:"):
                    self.after(
                        0,
                        self.status_var.set,
                        "DRAG PLAYBACK COMPLETE // Recorded path finished",
                    )
                    self.after(0, self._stop_runtime_timer)
                elif clean.startswith("** DRAG"):
                    self.after(
                        0,
                        self.status_var.set,
                        "FAULT // Drag Teach command failed",
                    )
                elif clean.startswith("MANUAL HAND COMPLETE:"):
                    self.after(0, self.status_var.set, "HAND COMPLETE // Manual Hand move finished")
                elif clean.startswith("MANUAL POLE COMPLETE:"):
                    self.after(0, self.status_var.set, "POLE COMPLETE // Manual Pole move finished")
                elif clean.startswith("** MANUAL HAND FAILED"):
                    self.after(0, self.status_var.set, "FAULT // Manual Hand move failed")
                elif clean.startswith("** MANUAL POLE FAILED"):
                    self.after(0, self.status_var.set, "FAULT // Manual Pole move failed")
                elif "VERIFIED:" in clean:
                    self.after(0, self.status_var.set, "VERIFIED // Trajectory stored successfully")
                elif "running (" in clean:
                    self.after(0, self.status_var.set, "MOTION ACTIVE // Robot program running")
                    self.after(0, self._start_runtime_timer)
                elif "RUN COMPLETE:" in clean:
                    self.after(0, self.status_var.set, "COMPLETE // Robot program finished")

                    # rm_upload.py prints: RUN COMPLETE: 18.7 s, no error
                    match = re.search(r"RUN COMPLETE:\s*([0-9]+(?:\.[0-9]+)?)\s*s", clean)
                    if match:
                        seconds = float(match.group(1))
                        self.after(0, self._stop_runtime_timer, seconds)
                    else:
                        self.after(0, self._stop_runtime_timer)
                elif "UPLOAD FAILED" in clean or "NOT VERIFIED" in clean:
                    self.after(0, self.status_var.set, "FAULT // Upload or verification failed")
                elif "RUN NOT COMPLETE" in clean or "NOT STARTED" in clean:
                    self.after(0, self.status_var.set, "STOPPED / FAULT // Run did not complete")
                    self.after(0, self._stop_runtime_timer)

            rc = self.process.wait()
            # Store this before scheduling any Tk callbacks. Page-02 dual
            # orchestration can now safely detect completion without racing
            # the asynchronously-updated status text.
            self.last_command_rc = rc

            if rc == 0:
                final = self.status_var.get()

                if "--list" in cmd:
                    # UI callbacks from parsed slot lines are queued on the Tk loop.
                    self.after(
                        50,
                        self.status_var.set,
                        "SYNC COMPLETE // Controller program list refreshed",
                    )
                elif not (
                    final.startswith("COMPLETE //")
                    or final.startswith("VERIFIED //")
                    or final.startswith("DRAG ")
                    or final.startswith("DRAG CONTROLLER ")
                    or final.startswith("MODE ")
                    or final.startswith("GLOBAL ")
                ):
                    self.after(
                        0,
                        self.status_var.set,
                        "COMPLETE // Command finished successfully",
                    )
            else:
                self.after(
                    0,
                    self.status_var.set,
                    f"FAULT // Command exited with code {rc}",
                )

        except Exception as exc:
            self.last_command_rc = -999
            self.after(0, self._append_log, f"\nUI ERROR: {exc}\n")
            self.after(0, self.status_var.set, "UI / command error")
        finally:
            self.process = None

            if is_global_sequence:
                try:
                    index = cmd.index("--global-sequence-file")
                    Path(cmd[index + 1]).unlink(missing_ok=True)
                except Exception:
                    pass

            if self.run_started_at is not None:
                self.after(0, self._stop_runtime_timer)
            self.motion_capable_command = False

            if is_drag_session:
                self.after(0, self._drag_session_finished)
            else:
                self.after(0, self._set_busy, False)


    def _slow_stop(self):
        p = self.process

        # During MULTI RUN, clicking SLOW STOP always cancels the remaining queue,
        # even in the short transition between two programs.
        if self.multi_run_active:
            self.sequence_cancel_requested = True

        if not p or p.poll() is not None:
            if self.multi_run_active:
                self.status_var.set("STOP REQUESTED // Remaining sequence cancelled")
                self._append_log("\n[UI] SLOW STOP clicked between programs — remaining sequence cancelled.\n")
                self.stop_btn.configure(state="disabled")
                self.quick_stop_btn.configure(state="disabled")
            else:
                self.status_var.set("IDLE // No active motion command")
            return

        # No confirmation by design: one click requests the slow stop immediately.
        self.status_var.set(
            "STOP REQUESTED // Active program stopping"
            + (" // Remaining sequence cancelled" if self.multi_run_active else "")
        )
        self._append_log("\n[UI] SLOW STOP clicked — sending Ctrl+C / SIGINT immediately...\n")

        # Disable the button immediately to avoid repeated stop signals.
        self.stop_btn.configure(state="disabled")
        self.quick_stop_btn.configure(state="disabled")

        try:
            if os.name == "nt":
                p.send_signal(signal.CTRL_BREAK_EVENT)
            else:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception as exc:
            self._append_log(f"[UI] Could not send slow-stop signal: {exc}\n")
            self.status_var.set("FAULT // Slow-stop signal failed")
            messagebox.showerror(
                "Slow Stop",
                "Could not send the software slow-stop signal.\n"
                "Use the robot's hardware E-stop if required.",
            )




class WorkflowSequencePanel(ttk.Frame):
    """Page-02 ordered workflow builder for one robot arm."""

    HAND_NAMES = (
        "Little", "Ring", "Middle", "Index", "Thumb Bend", "Thumb Rotate"
    )

    def __init__(self, parent, app, arm_panel, side):
        super().__init__(parent, style="App.TFrame")
        self.app = app
        self.arm_panel = arm_panel
        self.side = side
        self.side_name = side.upper()
        self.steps = []

        self.program_var = tk.StringVar()

        # Speed used ONLY for the controller's move into the stored
        # trajectory's first pose. The trajectory itself keeps stored timing.
        self.program_transit_speed_var = tk.StringVar(value="30")
        self.program_dual_rule_var = tk.StringVar(value="TOGETHER")

        self.global_var = tk.StringVar()
        self.global_move_var = tk.StringVar(value="MoveJ")
        self.global_speed_var = tk.StringVar(value="20")
        self.global_wait_var = tk.StringVar(value="0")

        self.pole_var = tk.StringVar(value="0")
        self.wait_var = tk.StringVar(value="1")

        self.hand_vars = [
            tk.StringVar(value="1000") for _ in range(6)
        ]

        self.status_var = tk.StringVar(
            value=f"{self.side_name} sequence ready"
        )

        self._build()
        self.refresh_sources()

    def _entry(self, parent, variable, width=8):
        return tk.Entry(
            parent,
            textvariable=variable,
            width=width,
            justify="center",
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 9, "bold"),
        )

    def _build(self):
        frame = ttk.LabelFrame(
            self,
            text=f"{self.side_name} ARM // SEQUENCE",
            padding=8,
            style="Tech.TLabelframe",
        )
        frame.pack(fill="both", expand=True)

        add_book = ttk.Notebook(
            frame,
            style="Robot.TNotebook",
        )
        add_book.pack(fill="x", pady=(0, 7))

        program_tab = ttk.Frame(add_book, padding=7, style="Panel.TFrame")
        global_tab = ttk.Frame(add_book, padding=7, style="Panel.TFrame")
        aux_tab = ttk.Frame(add_book, padding=7, style="Panel.TFrame")

        add_book.add(program_tab, text=" PROGRAM ")
        add_book.add(global_tab, text=" GLOBAL POINT ")
        add_book.add(aux_tab, text=" HAND / POLE / WAIT ")

        # PROGRAM ----------------------------------------------------------
        ttk.Label(
            program_tab,
            text="Stored Program",
            style="Panel.TLabel",
        ).grid(row=0, column=0, sticky="w", padx=(0, 5))

        self.program_combo = ttk.Combobox(
            program_tab,
            textvariable=self.program_var,
            state="normal",
            style="Program.TCombobox",
        )
        self.program_combo.grid(
            row=0, column=1, sticky="ew", padx=(0, 7)
        )

        ttk.Label(
            program_tab,
            text="Start %",
            style="Panel.TLabel",
        ).grid(row=0, column=2, sticky="w", padx=(0, 4))

        self.program_transit_speed_entry = self._entry(
            program_tab,
            self.program_transit_speed_var,
            width=5,
        )
        self.program_transit_speed_entry.grid(
            row=0, column=3, sticky="w", padx=(0, 7), ipady=3
        )

        ttk.Label(
            program_tab,
            text="Dual Rule",
            style="Panel.TLabel",
        ).grid(row=0, column=4, sticky="w", padx=(0, 4))

        self.program_dual_rule_combo = ttk.Combobox(
            program_tab,
            textvariable=self.program_dual_rule_var,
            values=("TOGETHER", "RUN FIRST", "WAIT OTHER"),
            state="readonly",
            width=12,
            style="Program.TCombobox",
        )
        self.program_dual_rule_combo.grid(
            row=0,
            column=5,
            sticky="w",
            padx=(0, 7),
        )

        self.add_program_btn = ttk.Button(
            program_tab,
            text="+ ADD PROGRAM",
            command=self._add_program,
            style="Run.TButton",
        )
        self.add_program_btn.grid(row=0, column=6, sticky="ew")

        ttk.Label(
            program_tab,
            text=(
                "Start % = speed used only to reach this Program's first pose "
                "(default 30%). Stored trajectory timing is unchanged. "
                "Dual Rule controls TOGETHER / RUN FIRST / WAIT OTHER."
            ),
            style="PanelMuted.TLabel",
        ).grid(
            row=1,
            column=0,
            columnspan=7,
            sticky="w",
            pady=(5, 0),
        )

        program_tab.columnconfigure(1, weight=1)

        # GLOBAL POINT -----------------------------------------------------
        ttk.Label(
            global_tab,
            text="Global Point",
            style="Panel.TLabel",
        ).grid(row=0, column=0, sticky="w", padx=(0, 5))

        self.global_combo = ttk.Combobox(
            global_tab,
            textvariable=self.global_var,
            state="normal",
            style="Program.TCombobox",
        )
        self.global_combo.grid(
            row=0, column=1, sticky="ew", padx=(0, 6)
        )

        self.global_move_combo = ttk.Combobox(
            global_tab,
            textvariable=self.global_move_var,
            values=("MoveJ", "MoveL"),
            state="readonly",
            width=7,
        )
        self.global_move_combo.grid(
            row=0, column=2, padx=(0, 6)
        )

        ttk.Label(
            global_tab,
            text="Speed",
            style="Panel.TLabel",
        ).grid(row=0, column=3, sticky="w", padx=(0, 3))

        self.global_speed_entry = self._entry(
            global_tab,
            self.global_speed_var,
            width=5,
        )
        self.global_speed_entry.grid(
            row=0, column=4, padx=(0, 6), ipady=3
        )

        ttk.Label(
            global_tab,
            text="Wait",
            style="Panel.TLabel",
        ).grid(row=0, column=5, sticky="w", padx=(0, 3))

        self.global_wait_entry = self._entry(
            global_tab,
            self.global_wait_var,
            width=5,
        )
        self.global_wait_entry.grid(
            row=0, column=6, padx=(0, 6), ipady=3
        )

        self.add_global_btn = ttk.Button(
            global_tab,
            text="+ ADD GLOBAL",
            command=self._add_global,
            style="Upload.TButton",
        )
        self.add_global_btn.grid(row=0, column=7, sticky="ew")
        global_tab.columnconfigure(1, weight=1)

        # AUX --------------------------------------------------------------
        aux_top = ttk.Frame(aux_tab, style="Panel.TFrame")
        aux_top.pack(fill="x")

        ttk.Label(
            aux_top,
            text="Pole mm",
            style="Panel.TLabel",
        ).pack(side="left")
        self.pole_entry = self._entry(
            aux_top,
            self.pole_var,
            width=6,
        )
        self.pole_entry.pack(side="left", padx=(5, 5), ipady=3)
        self.add_pole_btn = ttk.Button(
            aux_top,
            text="+ POLE",
            command=self._add_pole,
            style="Upload.TButton",
        )
        self.add_pole_btn.pack(side="left", padx=(0, 12))

        ttk.Label(
            aux_top,
            text="Wait s",
            style="Panel.TLabel",
        ).pack(side="left")
        self.wait_entry = self._entry(
            aux_top,
            self.wait_var,
            width=6,
        )
        self.wait_entry.pack(side="left", padx=(5, 5), ipady=3)
        self.add_wait_btn = ttk.Button(
            aux_top,
            text="+ WAIT",
            command=self._add_wait,
            style="List.TButton",
        )
        self.add_wait_btn.pack(side="left")

        hand_row = ttk.Frame(aux_tab, style="Panel.TFrame")
        hand_row.pack(fill="x", pady=(7, 0))

        for i, label in enumerate(("L", "R", "M", "I", "TB", "TR")):
            ttk.Label(
                hand_row,
                text=label,
                style="Panel.TLabel",
            ).pack(side="left", padx=(0, 2))

            entry = self._entry(
                hand_row,
                self.hand_vars[i],
                width=5,
            )
            entry.pack(side="left", padx=(0, 4), ipady=3)

        ttk.Button(
            hand_row,
            text="GRIP",
            command=lambda: self._set_hand(
                [0, 0, 0, 0, 150, 1000]
            ),
            style="Orange.TButton",
        ).pack(side="left", padx=(3, 3))

        ttk.Button(
            hand_row,
            text="OPEN",
            command=lambda: self._set_hand(
                [1000, 1000, 1000, 1000, 1000, 1000]
            ),
            style="Run.TButton",
        ).pack(side="left", padx=(0, 5))

        self.add_hand_btn = ttk.Button(
            hand_row,
            text="+ ADD HAND",
            command=self._add_hand,
            style="Run.TButton",
        )
        self.add_hand_btn.pack(side="right")

        # Sequence table ---------------------------------------------------
        table_frame = ttk.Frame(frame, style="Panel.TFrame")
        table_frame.pack(fill="both", expand=True)

        columns = ("num", "type", "target", "options")
        self.tree = ttk.Treeview(
            table_frame,
            columns=columns,
            show="headings",
            height=12,
            selectmode="browse",
        )
        self.tree.heading("num", text="#")
        self.tree.heading("type", text="TYPE")
        self.tree.heading("target", text="TARGET")
        self.tree.heading("options", text="OPTIONS")

        self.tree.column("num", width=38, anchor="center", stretch=False)
        self.tree.column("type", width=90, anchor="center", stretch=False)
        self.tree.column("target", width=190, anchor="w")
        self.tree.column("options", width=240, anchor="w")

        scroll = ttk.Scrollbar(
            table_frame,
            orient="vertical",
            command=self.tree.yview,
        )
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

        tools = ttk.Frame(frame, style="Panel.TFrame")
        tools.pack(fill="x", pady=(7, 5))

        for label, command in (
            ("↑ UP", self._up),
            ("↓ DOWN", self._down),
            ("DUPLICATE", self._duplicate),
            ("REMOVE", self._remove),
            ("CLEAR", self._clear),
        ):
            ttk.Button(
                tools,
                text=label,
                command=command,
            ).pack(side="left", padx=(0, 4))

        ttk.Button(
            tools,
            text="LOAD",
            command=self._load,
            style="List.TButton",
        ).pack(side="right", padx=(4, 0))

        ttk.Button(
            tools,
            text="SAVE",
            command=self._save,
            style="Upload.TButton",
        ).pack(side="right")

        run_bar = ttk.Frame(frame, style="Panel.TFrame")
        run_bar.pack(fill="x", pady=(5, 0))

        self.run_btn = ttk.Button(
            run_bar,
            text=f"▶ RUN {self.side_name} SEQUENCE",
            command=lambda: self.app._run_single_page2_sequence(self.side),
            style="Run.TButton",
        )
        self.run_btn.pack(side="left", fill="x", expand=True, padx=(0, 5))

        self.stop_btn = ttk.Button(
            run_bar,
            text=f"■ STOP {self.side_name}",
            command=lambda: self.app._stop_single_page2_sequence(self.side),
            state="disabled",
            style="Stop.TButton",
        )
        self.stop_btn.pack(side="left")

        ttk.Label(
            frame,
            textvariable=self.status_var,
            style="PanelMuted.TLabel",
        ).pack(fill="x", pady=(6, 0))

    def refresh_sources(self):
        try:
            self.program_combo.configure(
                values=self.arm_panel._program_combo_values()
            )
        except Exception:
            self.program_combo.configure(values=())

        names = sorted(
            self.arm_panel.global_points.keys(),
            key=lambda x: x.lower(),
        )
        self.global_combo.configure(values=names)

    def _parse_program(self):
        token = self.program_var.get().strip()
        match = re.match(r"^\s*(\d+)", token)
        if not match:
            messagebox.showerror(
                "Program",
                f"{self.side_name}: choose or enter a Program ID.",
                parent=self,
            )
            return None

        slot = int(match.group(1))
        if not 1 <= slot <= 100:
            messagebox.showerror(
                "Program",
                "Program ID must be 1-100.",
                parent=self,
            )
            return None

        try:
            transit_speed = int(
                float(self.program_transit_speed_var.get().strip())
            )
        except Exception:
            transit_speed = 30

        if not 1 <= transit_speed <= 100:
            messagebox.showerror(
                "Start / Transit speed",
                "Start / Transit speed must be 1-100%.",
                parent=self,
            )
            return None

        name = ""
        if slot in self.arm_panel.controller_programs:
            held = self.arm_panel.controller_programs[slot]
            if isinstance(held, tuple):
                name = str(held[0] or "")
            else:
                name = str(held or "")

        rule = self.program_dual_rule_var.get().strip().upper()
        if rule not in ("TOGETHER", "RUN FIRST", "WAIT OTHER"):
            rule = "TOGETHER"

        return {
            "type": "program",
            "id": slot,
            "name": name,

            # Legacy field kept for JSON compatibility. Stored trajectory
            # playback timing remains defined by the controller program.
            "speed": 100,

            # New v51 field: only the move into the first trajectory pose.
            "transit_speed": transit_speed,
            "dual_rule": rule,
        }

    def _add_program(self):
        step = self._parse_program()
        if step:
            self._append(step)

    def _add_global(self):
        name = self.global_var.get().strip()
        if not name:
            messagebox.showerror(
                "Global Point",
                f"{self.side_name}: choose or enter a global point name.",
                parent=self,
            )
            return

        move = self.global_move_var.get().strip().lower()
        if move not in ("movej", "movel"):
            move = "movej"

        try:
            speed = int(float(self.global_speed_var.get().strip()))
            wait_s = float(self.global_wait_var.get().strip() or 0)
        except Exception:
            messagebox.showerror(
                "Global Point",
                "Speed and Wait must be numeric.",
                parent=self,
            )
            return

        if not 1 <= speed <= 100:
            messagebox.showerror(
                "Global Point",
                "Speed must be 1-100%.",
                parent=self,
            )
            return

        if not 0 <= wait_s <= 3600:
            messagebox.showerror(
                "Global Point",
                "Wait must be 0-3600 seconds.",
                parent=self,
            )
            return

        self._append({
            "type": "global",
            "name": name,
            "move": move,
            "speed": speed,
            "wait_s": wait_s,
        })

    def _add_pole(self):
        try:
            mm = float(self.pole_var.get().strip())
        except Exception:
            mm = -1

        if not 0 <= mm <= 315:
            messagebox.showerror(
                "Pole",
                "Pole height must be 0-315 mm.",
                parent=self,
            )
            return

        self._append({"type": "pole", "mm": mm})

    def _add_wait(self):
        try:
            seconds = float(self.wait_var.get().strip())
        except Exception:
            seconds = -1

        if not 0 <= seconds <= 3600:
            messagebox.showerror(
                "Wait",
                "Wait must be 0-3600 seconds.",
                parent=self,
            )
            return

        self._append({"type": "wait", "seconds": seconds})

    def _set_hand(self, values):
        for var, value in zip(self.hand_vars, values):
            var.set(str(value))

    def _add_hand(self):
        values = []
        try:
            for var in self.hand_vars:
                value = int(float(var.get().strip()))
                if not 0 <= value <= 1000:
                    raise ValueError
                values.append(value)
        except Exception:
            messagebox.showerror(
                "Hand",
                "All six Hand values must be 0-1000.",
                parent=self,
            )
            return

        self._append({"type": "hand", "values": values})

    def _append(self, step):
        self.steps.append(step)
        self._refresh_table(select=len(self.steps) - 1)

    def _row(self, index, step):
        kind = step["type"]

        if kind == "program":
            target = f"Program {step['id']}"
            if step.get("name"):
                target += f" // {step['name']}"
            rule = str(step.get("dual_rule") or "TOGETHER").upper()
            transit_speed = int(
                step.get("transit_speed", step.get("start_speed", 30))
            )
            options = (
                f"Start {transit_speed}% // Stored timing // {rule}"
            )

        elif kind == "pole":
            target = f"{step['mm']:g} mm"
            options = ""

        elif kind == "hand":
            target = ",".join(str(v) for v in step["values"])
            options = "L,R,M,I,TB,TR"

        elif kind == "wait":
            target = f"{step['seconds']:g} s"
            options = ""

        else:
            target = step["name"]
            options = (
                f"{step['move'].upper()} // "
                f"Speed {step['speed']}% // "
                f"Wait {step['wait_s']:g}s"
            )

        return (index + 1, kind.upper(), target, options)

    def _refresh_table(self, select=None):
        for item in self.tree.get_children():
            self.tree.delete(item)

        for index, step in enumerate(self.steps):
            iid = str(index)
            self.tree.insert(
                "",
                "end",
                iid=iid,
                values=self._row(index, step),
            )

        if select is not None and 0 <= select < len(self.steps):
            iid = str(select)
            self.tree.selection_set(iid)
            self.tree.focus(iid)
            self.tree.see(iid)

        self.status_var.set(
            f"{self.side_name} // {len(self.steps)} sequence step(s)"
        )

    def _selected_index(self):
        sel = self.tree.selection()
        if not sel:
            return None
        try:
            return int(sel[0])
        except Exception:
            return None

    def _up(self):
        i = self._selected_index()
        if i is None or i <= 0:
            return
        self.steps[i - 1], self.steps[i] = self.steps[i], self.steps[i - 1]
        self._refresh_table(i - 1)

    def _down(self):
        i = self._selected_index()
        if i is None or i >= len(self.steps) - 1:
            return
        self.steps[i + 1], self.steps[i] = self.steps[i], self.steps[i + 1]
        self._refresh_table(i + 1)

    def _duplicate(self):
        i = self._selected_index()
        if i is None:
            return
        step = json.loads(json.dumps(self.steps[i]))
        self.steps.insert(i + 1, step)
        self._refresh_table(i + 1)

    def _remove(self):
        i = self._selected_index()
        if i is None:
            return
        self.steps.pop(i)
        self._refresh_table(
            min(i, len(self.steps) - 1) if self.steps else None
        )

    def _clear(self):
        if not self.steps:
            return
        if not messagebox.askyesno(
            "Clear sequence",
            f"Clear all {self.side_name} sequence steps?",
            parent=self,
        ):
            return
        self.steps.clear()
        self._refresh_table()

    def sequence_document(self):
        return {
            "format": "realman_workflow_sequence_v1",
            "side": self.side,
            "steps": self.steps,
        }

    def write_temp(self):
        if not self.steps:
            raise ValueError(f"{self.side_name} sequence is empty")

        folder = Path(tempfile.gettempdir()) / "realman_dual_sequence_ui"
        folder.mkdir(parents=True, exist_ok=True)
        path = folder / f"{self.side}_workflow_sequence.json"
        path.write_text(
            json.dumps(self.sequence_document(), indent=2),
            encoding="utf-8",
        )
        return path

    def _save(self):
        if not self.steps:
            messagebox.showinfo(
                "Save Sequence",
                f"{self.side_name} sequence is empty.",
                parent=self,
            )
            return

        path = filedialog.asksaveasfilename(
            title=f"Save {self.side_name} workflow sequence",
            initialdir=str(Path.home() / "Downloads"),
            initialfile=f"{self.side}_workflow_sequence.json",
            defaultextension=".json",
            filetypes=[("JSON sequence", "*.json"), ("All files", "*.*")],
            parent=self,
        )
        if not path:
            return

        Path(path).write_text(
            json.dumps(self.sequence_document(), indent=2),
            encoding="utf-8",
        )

    def _load(self):
        path = filedialog.askopenfilename(
            title=f"Load {self.side_name} workflow sequence",
            initialdir=str(Path.home() / "Downloads"),
            filetypes=[("JSON sequence", "*.json"), ("All files", "*.*")],
            parent=self,
        )
        if not path:
            return

        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            if data.get("format") != "realman_workflow_sequence_v1":
                raise ValueError("wrong sequence format")
            steps = data.get("steps")
            if not isinstance(steps, list):
                raise ValueError("steps must be a list")

            # v51 backward compatibility:
            # old workflow JSON has only "speed". That old value was used as
            # the move-to-first-point speed. For safer starts, old Program
            # rows now receive a dedicated 30% transit speed unless they
            # already contain transit_speed/start_speed.
            for step in steps:
                if not isinstance(step, dict):
                    continue
                if step.get("type") != "program":
                    continue
                if (
                    "transit_speed" not in step
                    and "start_speed" not in step
                ):
                    step["transit_speed"] = 30
        except Exception as exc:
            messagebox.showerror(
                "Load Sequence",
                f"Could not load sequence:\n\n{exc}",
                parent=self,
            )
            return

        self.steps = steps
        self._refresh_table(0 if self.steps else None)

    def set_running(self, running):
        self.run_btn.configure(
            state="disabled" if running else "normal"
        )
        self.stop_btn.configure(
            state="normal" if running else "disabled"
        )
        if running:
            self.status_var.set(
                f"{self.side_name} // sequence running..."
            )


class DualArmTrajectoryUI(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("RealMan // Dual-Arm Operations Console")
        self.geometry("1880x1000")
        self.minsize(1320, 760)
        self.configure(bg=BG)

        self._apply_robot_theme()

        here = Path(__file__).resolve().parent

        # Prefer the latest runner used by v20.
        runner_candidates = [
            here / "rm_upload_fast_v14_safe_transit.py",
            here / "rm_upload_fast_v13_overall_speed.py",
            here / "rm_upload_fast_v12_fast_workflow.py",
            here / "rm_upload_fast_v11_workflow_sequence.py",
            here / "rm_upload_fast_v10_six_finger.py",
            here / "rm_upload_fast_v9_global_points.py",
            here / "rm_upload_fast_v8_run_mode.py",
            here / "rm_upload_fast_v7_drag_controller.py",
            here / "rm_upload_fast_v6_drag_teach.py",
            here / "rm_upload_fast_v5.py",
            here / "rm_upload_fast_v4.py",
            here / "rm_upload_fast_v3.py",
            here / "rm_upload_fast_v2.py",
            here / "rm_upload_fast.py",
            here / "rm_upload.py",
        ]
        default_rm_upload = next(
            (p for p in runner_candidates if p.exists()),
            runner_candidates[-1],
        )

        downloads = Path.home() / "Downloads"
        self.default_track_dir = (
            downloads if downloads.is_dir() else (here / "plan_txt_files")
        )

        # 01 // Shared trajectory setup.
        self.rm_upload_var = tk.StringVar(
            value=str(default_rm_upload) if default_rm_upload.exists() else ""
        )
        self.sdk_var = tk.StringVar(value=DEFAULT_SDK)

        # Shared version banner shown above both top-level pages.
        self.version_banner_var = tk.StringVar()
        self.rm_upload_var.trace_add(
            "write",
            self._on_runner_path_changed,
        )
        self._refresh_version_banner()
        # Separate trajectory selections and upload slots for each arm.
        self.left_track_var = tk.StringVar()
        self.right_track_var = tk.StringVar()
        self.left_slot_var = tk.StringVar(value="25")
        self.right_slot_var = tk.StringVar(value="25")

        self.left_panel = None
        self.right_panel = None

        # Synchronized one-program-per-arm run state.
        self.dual_run_active = False
        self.dual_run_started_at = None
        self.dual_run_after_id = None
        self.dual_status_var = tk.StringVar(
            value="READY // Select one stored program for each arm"
        )
        self.dual_runtime_var = tk.StringVar(value="0.0 s")

        # Common dual-arm Drag Teach state.
        self.dual_drag_active = False
        self.dual_drag_after_id = None

        # After STOP BOTH DRAG, show one popup for the two .txt files.
        self.dual_drag_save_pending = False
        self.dual_drag_save_dialog = None
        self.dual_drag_save_active = False
        self.dual_drag_save_after_id = None
        self.dual_drag_left_save_var = tk.StringVar()
        self.dual_drag_right_save_var = tk.StringVar()

        # Common RUN BOTH .TXT state.
        self.dual_file_run_dialog = None
        self.dual_file_run_active = False
        self.dual_file_run_after_id = None
        self.dual_file_left_var = tk.StringVar()
        self.dual_file_right_var = tk.StringVar()
        self.dual_file_left_slot_var = tk.StringVar(value="99")
        self.dual_file_right_slot_var = tk.StringVar(value="99")

        # PAGE 02 // Dual workflow sequence state.
        self.page2_dual_active = False
        self.page2_dual_after_id = None
        self.page2_dual_started_at = None
        self.page2_status_var = tk.StringVar(
            value="PAGE 02 READY // Build LEFT and RIGHT sequences"
        )
        self.page2_runtime_var = tk.StringVar(value="0.0 s")
        self.page2_overall_speed_var = tk.StringVar(value="100")

        self.page2_left_live_var = tk.StringVar(value="LEFT // IDLE")
        self.page2_right_live_var = tk.StringVar(value="RIGHT // IDLE")

        # Page 02 dual execution mode:
        # PARALLEL, LEFT -> RIGHT, or RIGHT -> LEFT.
        self.page2_serial_first_side = None
        self.page2_serial_second_side = None
        self.page2_serial_second_cmd = None

        # Program-by-program alternating mode.
        self.page2_alt_confirm_each_var = tk.BooleanVar(value=True)
        self.page2_alt_queue = []
        self.page2_alt_index = 0
        self.page2_alt_current = None

        self.page2_rule_plan = []
        self.page2_rule_index = 0
        self.page2_rule_stage = None
        self.page2_rule_current = None

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _apply_robot_theme(self):
        """Dark, high-contrast robotics / control-console theme."""
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        # Base widgets
        style.configure(".", font=("DejaVu Sans", 10))
        style.configure("TFrame", background=BG)
        style.configure("App.TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL)
        style.configure("Panel2.TFrame", background=PANEL_2)

        style.configure(
            "TLabel",
            background=BG,
            foreground=TEXT,
        )
        style.configure(
            "Panel.TLabel",
            background=PANEL,
            foreground=TEXT,
        )
        style.configure(
            "Muted.TLabel",
            background=BG,
            foreground=MUTED,
        )
        style.configure(
            "PanelMuted.TLabel",
            background=PANEL,
            foreground=MUTED,
        )
        style.configure(
            "Title.TLabel",
            background=BG,
            foreground=CYAN,
            font=("DejaVu Sans", 19, "bold"),
        )
        style.configure(
            "Kicker.TLabel",
            background=BG,
            foreground=PURPLE,
            font=("DejaVu Sans", 9, "bold"),
        )
        style.configure(
            "System.TLabel",
            background=BG,
            foreground=GREEN,
            font=("DejaVu Sans", 10, "bold"),
        )
        style.configure(
            "StatusTitle.TLabel",
            background=PANEL,
            foreground=CYAN,
            font=("DejaVu Sans", 10, "bold"),
        )
        style.configure(
            "StatusValue.TLabel",
            background=PANEL,
            foreground=TEXT,
            font=("DejaVu Sans", 11, "bold"),
        )
        style.configure(
            "RuntimeValue.TLabel",
            background=PANEL,
            foreground=GREEN,
            font=("DejaVu Sans Mono", 12, "bold"),
        )

        # Cards / frames
        style.configure(
            "Tech.TLabelframe",
            background=PANEL,
            bordercolor=BORDER,
            borderwidth=1,
            relief="solid",
        )
        style.configure(
            "Tech.TLabelframe.Label",
            background=BG,
            foreground=CYAN,
            font=("DejaVu Sans", 10, "bold"),
        )

        # Inputs
        style.configure(
            "TEntry",
            fieldbackground=ENTRY_BG,
            foreground=TEXT,
            bordercolor=BORDER,
            lightcolor=BORDER,
            darkcolor=BORDER,
            padding=6,
        )
        style.map(
            "TEntry",
            bordercolor=[("focus", CYAN)],
            lightcolor=[("focus", CYAN)],
            darkcolor=[("focus", CYAN)],
        )

        style.configure(
            "TCombobox",
            fieldbackground=ENTRY_BG,
            background=PANEL_2,
            foreground=TEXT,
            arrowcolor=CYAN,
            bordercolor=BORDER,
            padding=5,
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", ENTRY_BG)],
            foreground=[("readonly", TEXT)],
            selectbackground=[("readonly", ENTRY_BG)],
            selectforeground=[("readonly", TEXT)],
        )

        # Editable program selector: use a slightly lighter focus field so
        # the native ttk caret remains visible across Linux themes.
        style.configure(
            "Program.TCombobox",
            fieldbackground=ENTRY_BG,
            background=PANEL_2,
            foreground=TEXT,
            arrowcolor=CYAN,
            bordercolor=BORDER,
            padding=5,
        )
        style.map(
            "Program.TCombobox",
            fieldbackground=[
                ("focus", "#15324B"),
                ("!focus", ENTRY_BG),
            ],
            foreground=[
                ("focus", "#FFFFFF"),
                ("!focus", TEXT),
            ],
            selectbackground=[("focus", BLUE)],
            selectforeground=[("focus", "#FFFFFF")],
            bordercolor=[("focus", CYAN), ("!focus", BORDER)],
        )

        style.configure(
            "TSpinbox",
            fieldbackground=ENTRY_BG,
            background=PANEL_2,
            foreground=TEXT,
            arrowcolor=CYAN,
            bordercolor=BORDER,
            padding=5,
        )

        # Default small navigation button
        style.configure(
            "TButton",
            background=PANEL_2,
            foreground=TEXT,
            bordercolor=BORDER,
            padding=(10, 6),
            font=("DejaVu Sans", 9, "bold"),
        )
        style.map(
            "TButton",
            background=[("active", "#173754"), ("disabled", "#102030")],
            foreground=[("disabled", "#587084")],
        )

        def button_style(name, bg, active):
            style.configure(
                name,
                background=bg,
                foreground="#FFFFFF",
                bordercolor=bg,
                focusthickness=1,
                focuscolor=CYAN,
                padding=(14, 9),
                font=("DejaVu Sans", 10, "bold"),
            )
            style.map(
                name,
                background=[("active", active), ("disabled", "#253342")],
                foreground=[("disabled", "#718292")],
            )

        button_style("Upload.TButton", PURPLE, "#A78BFA")
        button_style("Run.TButton", GREEN, "#17E39B")
        button_style("List.TButton", BLUE, "#60A5FA")
        button_style("Stop.TButton", RED, "#FF647D")
        button_style("Orange.TButton", ORANGE, "#FFB84D")

        def compact_button_style(name, bg, active):
            style.configure(
                name,
                background=bg,
                foreground="#FFFFFF",
                bordercolor=bg,
                padding=(9, 5),
                font=("DejaVu Sans", 8, "bold"),
            )
            style.map(
                name,
                background=[
                    ("active", active),
                    ("disabled", "#253342"),
                ],
                foreground=[("disabled", "#718292")],
            )

        compact_button_style("Compact.TButton", PANEL_2, "#173754")
        compact_button_style("CompactBlue.TButton", BLUE, "#60A5FA")
        compact_button_style("CompactGreen.TButton", GREEN, "#17E39B")
        compact_button_style("CompactOrange.TButton", ORANGE, "#FFB84D")
        compact_button_style("CompactStop.TButton", RED, "#FF647D")

        style.configure(
            "ArmTitle.TLabel",
            background=PANEL_2,
            foreground=CYAN,
            font=("DejaVu Sans", 13, "bold"),
        )
        style.configure(
            "ArmIp.TLabel",
            background=PANEL_2,
            foreground=TEXT,
            font=("DejaVu Sans Mono", 10, "bold"),
        )
        style.configure(
            "ModeBadge.TLabel",
            background=PANEL_2,
            foreground=GREEN,
            font=("DejaVu Sans", 9, "bold"),
        )

        style.configure(
            "Robot.TNotebook",
            background=BG,
            borderwidth=0,
            tabmargins=(0, 0, 0, 0),
        )
        style.configure(
            "Robot.TNotebook.Tab",
            background=PANEL_2,
            foreground=MUTED,
            bordercolor=BORDER,
            padding=(13, 8),
            font=("DejaVu Sans", 9, "bold"),
        )
        style.map(
            "Robot.TNotebook.Tab",
            background=[
                ("selected", PANEL),
                ("active", "#173754"),
            ],
            foreground=[
                ("selected", CYAN),
                ("active", "#FFFFFF"),
            ],
        )

        # File browser / lists
        style.configure(
            "Treeview",
            background=ENTRY_BG,
            fieldbackground=ENTRY_BG,
            foreground=TEXT,
            bordercolor=BORDER,
            rowheight=28,
        )
        style.map(
            "Treeview",
            background=[("selected", BLUE)],
            foreground=[("selected", "#FFFFFF")],
        )
        style.configure(
            "Treeview.Heading",
            background=PANEL_2,
            foreground=CYAN,
            bordercolor=BORDER,
            font=("DejaVu Sans", 9, "bold"),
            padding=(5, 6),
        )
        style.map(
            "Treeview.Heading",
            background=[("active", "#173754")],
        )

        style.configure(
            "Vertical.TScrollbar",
            background=PANEL_2,
            troughcolor=BG,
            arrowcolor=CYAN,
            bordercolor=BG,
        )

        # Native Tk text fields: bright blinking insertion cursor.
        self.option_add("*Entry.insertBackground", "#FFFFFF")
        self.option_add("*Entry.insertWidth", 3)
        self.option_add("*Spinbox.insertBackground", "#FFFFFF")
        self.option_add("*Spinbox.insertWidth", 3)
        self.option_add("*Text.insertBackground", "#FFFFFF")
        self.option_add("*Text.insertWidth", 3)


    def _tech_entry(self, parent, textvariable=None, width=None):
        """Dark text entry with a bright, clearly visible blinking caret."""
        kwargs = {
            "textvariable": textvariable,
            "bg": ENTRY_BG,
            "fg": TEXT,
            "insertbackground": "#FFFFFF",
            "insertwidth": 3,
            "selectbackground": BLUE,
            "selectforeground": "#FFFFFF",
            "relief": "flat",
            "highlightthickness": 1,
            "highlightbackground": BORDER,
            "highlightcolor": CYAN,
            "font": ("DejaVu Sans", 10),
        }
        if width is not None:
            kwargs["width"] = width

        entry = tk.Entry(parent, **kwargs)

        # Make focus very obvious in the dark UI.
        def on_focus_in(_event):
            entry.configure(
                highlightbackground=CYAN,
                highlightcolor=CYAN,
            )

        def on_focus_out(_event):
            entry.configure(
                highlightbackground=BORDER,
                highlightcolor=CYAN,
            )

        entry.bind("<FocusIn>", on_focus_in, add="+")
        entry.bind("<FocusOut>", on_focus_out, add="+")
        return entry


    def _tech_spinbox(self, parent, textvariable=None, from_=1, to=100, width=8):
        """Dark Spinbox with a bright white blinking insertion cursor."""
        spin = tk.Spinbox(
            parent,
            from_=from_,
            to=to,
            textvariable=textvariable,
            width=width,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            buttonbackground=PANEL_2,
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 10),
        )

        # Make the focused upload-slot field obvious.
        def on_focus_in(_event):
            spin.configure(
                highlightbackground=CYAN,
                highlightcolor=CYAN,
                insertbackground="#FFFFFF",
            )

        def on_focus_out(_event):
            spin.configure(
                highlightbackground=BORDER,
                highlightcolor=CYAN,
            )

        spin.bind("<FocusIn>", on_focus_in, add="+")
        spin.bind("<FocusOut>", on_focus_out, add="+")
        return spin



    def _runner_version_from_path(self):
        raw = self.rm_upload_var.get().strip()
        if not raw:
            return EXPECTED_RUNNER_VERSION, "(runner not selected)"

        name = Path(raw).name
        match = re.search(r"_v(\d+)", name, re.IGNORECASE)

        if match:
            version = f"v{match.group(1)}"
        else:
            version = "version unknown"

        return version, name

    def _refresh_version_banner(self):
        runner_version, runner_name = self._runner_version_from_path()
        python_version = (
            f"{sys.version_info.major}."
            f"{sys.version_info.minor}."
            f"{sys.version_info.micro}"
        )

        ui_name = Path(__file__).name

        self.version_banner_var.set(
            f"UI PROGRAM {UI_PROGRAM_VERSION}  //  {ui_name}"
            f"    •    RUNNER {runner_version}  //  {runner_name}"
            f"    •    PYTHON {python_version}"
        )

    def _on_runner_path_changed(self, *_args):
        self._refresh_version_banner()

    def _build_ui(self):
        # --------------------------------------------------------------
        # Persistent software version banner
        # Visible while using PAGE 01 and PAGE 02.
        # --------------------------------------------------------------
        version_bar = ttk.Frame(
            self,
            padding=(12, 6),
            style="Panel2.TFrame",
        )
        version_bar.pack(fill="x")

        ttk.Label(
            version_bar,
            text="SOFTWARE VERSION",
            style="StatusTitle.TLabel",
        ).pack(side="left", padx=(0, 10))

        ttk.Label(
            version_bar,
            textvariable=self.version_banner_var,
            style="PanelMuted.TLabel",
        ).pack(side="left", fill="x", expand=True)

        ttk.Label(
            version_bar,
            text="● DUAL ARM",
            style="System.TLabel",
        ).pack(side="right", padx=(10, 0))

        self.main_pages = ttk.Notebook(
            self,
            style="Robot.TNotebook",
        )
        self.main_pages.pack(fill="both", expand=True)

        page1 = ttk.Frame(
            self.main_pages,
            style="App.TFrame",
        )
        page2 = ttk.Frame(
            self.main_pages,
            style="App.TFrame",
        )

        self.main_pages.add(
            page1,
            text="  PAGE 01 // MANUAL + SETUP  ",
        )
        self.main_pages.add(
            page2,
            text="  PAGE 02 // DUAL SEQUENCE  ",
        )

        root = ttk.Frame(page1, padding=12, style="App.TFrame")
        root.pack(fill="both", expand=True)

        # --------------------------------------------------------------
        # Header
        # --------------------------------------------------------------
        header = ttk.Frame(root, style="App.TFrame")
        header.pack(fill="x", pady=(0, 6))

        header_left = ttk.Frame(header, style="App.TFrame")
        header_left.pack(side="left", fill="x", expand=True)

        ttk.Label(
            header_left,
            text="REALMAN // DUAL ROBOTICS CONTROL",
            style="Kicker.TLabel",
        ).pack(anchor="w")

        ttk.Label(
            header_left,
            text="Dual-Arm Operations Console",
            style="Title.TLabel",
        ).pack(anchor="w", pady=(2, 2))

        ttk.Label(
            header_left,
            text=(
                "LEFT 192.168.1.10   •   RIGHT 192.168.1.103   •   "
                "Task tabs • independent control • synchronized dual run"
            ),
            style="Muted.TLabel",
        ).pack(anchor="w")

        header_right = ttk.Frame(
            header,
            style="App.TFrame",
        )
        header_right.pack(side="right", anchor="ne", pady=(3, 0))

        ttk.Label(
            header_right,
            text="●  DUAL SYSTEM",
            style="System.TLabel",
        ).pack(anchor="e")

        dual_drag_tools = ttk.Frame(
            header_right,
            style="App.TFrame",
        )
        dual_drag_tools.pack(anchor="e", pady=(5, 0))

        self.dual_drag_start_btn = ttk.Button(
            dual_drag_tools,
            text="✋ START BOTH DRAG",
            command=self._start_both_drag,
            style="CompactOrange.TButton",
        )
        self.dual_drag_start_btn.pack(side="left", padx=(0, 5))

        self.dual_drag_stop_btn = ttk.Button(
            dual_drag_tools,
            text="■ STOP BOTH DRAG",
            command=self._stop_both_drag,
            state="disabled",
            style="CompactStop.TButton",
        )
        self.dual_drag_stop_btn.pack(side="left", padx=(0, 5))

        self.dual_file_run_btn = ttk.Button(
            dual_drag_tools,
            text="▶ RUN BOTH .TXT",
            command=self._show_dual_file_run_popup,
            style="CompactGreen.TButton",
        )
        self.dual_file_run_btn.pack(side="left")

        # --------------------------------------------------------------
        # Prepare the 50 / 50 arm workspace first. It is packed after the
        # compact 30 / 70 top control row.
        # --------------------------------------------------------------
        ttk.Label(
            root,
            text=(
                "RUN BOTH follows TOGETHER / RUN FIRST / WAIT OTHER rules. "
                "Each Program uses its Start % only for the move into the first pose; "
                "stored trajectory timing is unchanged."
            ),
            style="Muted.TLabel",
        ).pack(fill="x", pady=(0, 6))

        split = ttk.Frame(root, style="App.TFrame")

        split.columnconfigure(0, weight=1, uniform="arms")
        split.columnconfigure(1, weight=1, uniform="arms")
        split.rowconfigure(0, weight=1)

        left_holder = ttk.Frame(split, style="App.TFrame")
        right_holder = ttk.Frame(split, style="App.TFrame")

        left_holder.grid(
            row=0, column=0, sticky="nsew", padx=(0, 5)
        )
        right_holder.grid(
            row=0, column=1, sticky="nsew", padx=(5, 0)
        )

        self.left_panel = ArmControlPanel(
            left_holder,
            side="left",
            ip_address="192.168.1.10",
            rm_upload_var=self.rm_upload_var,
            sdk_var=self.sdk_var,
            track_var=self.left_track_var,
            slot_var=self.left_slot_var,
            default_track_dir=self.default_track_dir,
        )
        self.left_panel.pack(fill="both", expand=True)

        self.right_panel = ArmControlPanel(
            right_holder,
            side="right",
            ip_address="192.168.1.103",
            rm_upload_var=self.rm_upload_var,
            sdk_var=self.sdk_var,
            track_var=self.right_track_var,
            slot_var=self.right_slot_var,
            default_track_dir=self.default_track_dir,
        )
        self.right_panel.pack(fill="both", expand=True)

        # --------------------------------------------------------------
        # COMPACT TOP ROW // 30% SYSTEM + 70% DUAL RUN
        # --------------------------------------------------------------
        top_controls = ttk.Frame(root, style="App.TFrame")
        top_controls.pack(fill="x", pady=(0, 7))

        # Explicit 30 / 70 proportion.
        top_controls.columnconfigure(0, weight=3, uniform="top_controls")
        top_controls.columnconfigure(1, weight=7, uniform="top_controls")

        # --------------------------------------------------------------
        # 30% // Shared runner + SDK
        # --------------------------------------------------------------
        setup = ttk.LabelFrame(
            top_controls,
            text="SYSTEM // RUNNER + SDK",
            padding=(7, 4),
            style="Tech.TLabelframe",
        )
        setup.grid(
            row=0,
            column=0,
            sticky="nsew",
            padx=(0, 4),
        )

        ttk.Label(
            setup,
            text="Runner",
            style="Panel.TLabel",
        ).grid(row=0, column=0, sticky="w", padx=(0, 4), pady=2)

        rm_entry = self._tech_entry(
            setup,
            textvariable=self.rm_upload_var,
            width=13,
        )
        rm_entry.grid(
            row=0,
            column=1,
            sticky="ew",
            pady=2,
            ipady=2,
        )

        ttk.Button(
            setup,
            text="...",
            width=3,
            command=self._browse_rm_upload,
            style="Compact.TButton",
        ).grid(row=0, column=2, padx=(4, 8), pady=2)

        ttk.Label(
            setup,
            text="SDK",
            style="Panel.TLabel",
        ).grid(row=0, column=3, sticky="w", padx=(0, 4), pady=2)

        sdk_entry = self._tech_entry(
            setup,
            textvariable=self.sdk_var,
            width=13,
        )
        sdk_entry.grid(
            row=0,
            column=4,
            sticky="ew",
            pady=2,
            ipady=2,
        )

        ttk.Button(
            setup,
            text="...",
            width=3,
            command=self._browse_sdk,
            style="Compact.TButton",
        ).grid(row=0, column=5, padx=(4, 0), pady=2)

        setup.columnconfigure(1, weight=1)
        setup.columnconfigure(4, weight=1)

        # --------------------------------------------------------------
        # 70% // Dual Run - one compact line
        # --------------------------------------------------------------
        dual = ttk.LabelFrame(
            top_controls,
            text="DUAL RUN // SYNCHRONIZED STORED PROGRAMS",
            padding=(7, 4),
            style="Tech.TLabelframe",
        )
        dual.grid(
            row=0,
            column=1,
            sticky="nsew",
            padx=(4, 0),
        )

        ttk.Label(
            dual,
            text="L",
            style="Panel.TLabel",
        ).grid(row=0, column=0, sticky="w", padx=(0, 4), pady=2)

        self.dual_left_combo = ttk.Combobox(
            dual,
            textvariable=self.left_panel.stored_program_var,
            state="normal",
            width=18,
            style="Program.TCombobox",
        )
        self.dual_left_combo.grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(0, 7),
            pady=2,
        )
        self.dual_left_combo.bind(
            "<Button-1>",
            lambda _e: self._refresh_dual_combo_values(),
            add="+",
        )

        ttk.Label(
            dual,
            text="R",
            style="Panel.TLabel",
        ).grid(row=0, column=2, sticky="w", padx=(0, 4), pady=2)

        self.dual_right_combo = ttk.Combobox(
            dual,
            textvariable=self.right_panel.stored_program_var,
            state="normal",
            width=18,
            style="Program.TCombobox",
        )
        self.dual_right_combo.grid(
            row=0,
            column=3,
            sticky="ew",
            padx=(0, 7),
            pady=2,
        )
        self.dual_right_combo.bind(
            "<Button-1>",
            lambda _e: self._refresh_dual_combo_values(),
            add="+",
        )

        self.dual_sync_btn = ttk.Button(
            dual,
            text="↻ SYNC",
            command=self._sync_both_program_lists,
            style="CompactBlue.TButton",
        )
        self.dual_sync_btn.grid(
            row=0,
            column=4,
            padx=3,
            pady=2,
            sticky="ew",
        )

        self.dual_run_btn = ttk.Button(
            dual,
            text="▶▶ RUN BOTH",
            command=self._run_both_programs,
            style="CompactGreen.TButton",
        )
        self.dual_run_btn.grid(
            row=0,
            column=5,
            padx=3,
            pady=2,
            sticky="ew",
        )

        self.dual_stop_btn = ttk.Button(
            dual,
            text="■ STOP BOTH",
            command=self._stop_both,
            state="disabled",
            style="CompactStop.TButton",
        )
        self.dual_stop_btn.grid(
            row=0,
            column=6,
            padx=3,
            pady=2,
            sticky="ew",
        )

        ttk.Label(
            dual,
            textvariable=self.dual_status_var,
            width=22,
            anchor="w",
            style="PanelMuted.TLabel",
        ).grid(
            row=0,
            column=7,
            sticky="ew",
            padx=(6, 4),
            pady=2,
        )

        ttk.Label(
            dual,
            textvariable=self.dual_runtime_var,
            style="RuntimeValue.TLabel",
        ).grid(
            row=0,
            column=8,
            sticky="e",
            padx=(4, 0),
            pady=2,
        )

        dual.columnconfigure(1, weight=1)
        dual.columnconfigure(3, weight=1)
        dual.columnconfigure(7, weight=1)

        self._refresh_dual_combo_values()

        # Main LEFT / RIGHT workspace receives all remaining vertical space.
        split.pack(fill="both", expand=True)

        # Page 01 keeps the existing workflow tabs and controls.
        self._build_sequence_page(page2)

    def _start_both_drag(self):
        """
        Start Drag Teach + recording on LEFT and RIGHT with synchronized
        subprocess launch. Each runner calls rm_start_drag_teach(1).
        """
        if self.dual_file_run_active:
            messagebox.showwarning(
                "RUN BOTH .TXT active",
                "Wait until the current .txt run is complete.",
                parent=self,
            )
            return

        if self.dual_drag_active:
            messagebox.showwarning(
                "Dual Drag active",
                "Both-arm Drag Teach is already active.",
                parent=self,
            )
            return

        if self.dual_run_active:
            messagebox.showwarning(
                "Dual Run active",
                "Stop/finish Dual Run before starting Drag Teach.",
                parent=self,
            )
            return

        busy = []
        if self.left_panel.has_active_command():
            busy.append("LEFT")
        if self.right_panel.has_active_command():
            busy.append("RIGHT")

        if busy:
            messagebox.showwarning(
                "Arm busy",
                "Both arms must be idle before starting common Drag Teach.\n\n"
                "Busy: " + ", ".join(busy),
                parent=self,
            )
            return

        left_env = self.left_panel._validate_environment()
        if not left_env:
            return

        right_env = self.right_panel._validate_environment()
        if not right_env:
            return

        ok = messagebox.askyesno(
            "Start BOTH arms Drag Teach",
            "LEFT AND RIGHT ARMS WILL ENTER DRAG TEACH MODE TOGETHER.\n\n"
            "Both arms can become hand-guidable/free and both controllers "
            "will record their Drag Teach trajectories.\n\n"
            "Make sure both arms/tools are supported safely, both work areas "
            "are clear, and both hardware E-stops are accessible.\n\n"
            "START BOTH DRAG + RECORD?",
            icon="warning",
            parent=self,
        )
        if not ok:
            return

        left_rm, left_sdk = left_env
        right_rm, right_sdk = right_env

        left_cmd = self.left_panel._base_command(left_rm, left_sdk) + [
            "--drag-session",
            "--side",
            "left",
            "--yes",
        ]
        right_cmd = self.right_panel._base_command(right_rm, right_sdk) + [
            "--drag-session",
            "--side",
            "right",
            "--yes",
        ]

        # Prepare both panel states before launching.
        for panel in (self.left_panel, self.right_panel):
            panel.drag_teach_active = True
            panel.drag_teach_has_recording = False
            panel.drag_toggle_btn.configure(
                text="■ STOP DRAG",
                state="normal",
            )
            panel.drag_save_btn.configure(state="disabled")
            panel.drag_playback_btn.configure(state="disabled")

        launch_barrier = threading.Barrier(2)

        self.dual_drag_save_pending = False
        self.dual_drag_active = True
        self.dual_drag_start_btn.configure(state="disabled")
        self.dual_drag_stop_btn.configure(state="normal")
        self.dual_drag_stop_btn.update_idletasks()

        try:
            self.left_panel._start_process_synced(
                left_cmd,
                "DUAL DRAG // LEFT starting",
                launch_barrier,
            )
            self.right_panel._start_process_synced(
                right_cmd,
                "DUAL DRAG // RIGHT starting",
                launch_barrier,
            )
        except Exception as exc:
            self.dual_drag_active = False
            self.dual_drag_start_btn.configure(state="normal")
            self.dual_drag_stop_btn.configure(state="disabled")

            self.left_panel._slow_stop()
            self.right_panel._slow_stop()

            messagebox.showerror(
                "Dual Drag Teach",
                f"Could not start both Drag Teach sessions:\n\n{exc}",
                parent=self,
            )
            return

        self._monitor_both_drag()

    def _monitor_both_drag(self):
        if not self.dual_drag_active:
            return

        # IMPORTANT:
        # Do not use has_active_command() here. Immediately after START BOTH
        # there is a short period where the worker threads are alive but
        # panel.process has not yet been assigned by subprocess.Popen().
        # Using has_active_command() during that window made the UI think
        # Drag Teach had already ended and disabled STOP BOTH DRAG.
        left_active = bool(self.left_panel.drag_teach_active)
        right_active = bool(self.right_panel.drag_teach_active)

        if left_active or right_active:
            self.dual_drag_stop_btn.configure(state="normal")
            self.dual_drag_start_btn.configure(state="disabled")
            self.dual_drag_after_id = self.after(
                150,
                self._monitor_both_drag,
            )
            return

        self.dual_drag_active = False
        self.dual_drag_after_id = None
        self.dual_drag_start_btn.configure(state="normal")
        self.dual_drag_stop_btn.configure(state="disabled")

        if self.dual_drag_save_pending:
            self.dual_drag_save_pending = False
            self.after(100, self._show_dual_drag_save_popup)

    def _stop_panel_drag_from_common(self, panel, retry=0):
        """
        Stop one Drag Teach session without another confirmation.
        A tiny retry covers the short window before Popen assigns panel.process.
        """
        if not panel.drag_teach_active:
            return

        process = panel.process

        if process is None or process.poll() is not None:
            if retry < 20 and panel.drag_teach_active:
                self.after(
                    100,
                    lambda p=panel, r=retry + 1:
                    self._stop_panel_drag_from_common(p, r),
                )
            return

        # _drag_teach_toggle() is already the tested one-click STOP path.
        panel._drag_teach_toggle()

    def _stop_both_drag(self):
        """
        Immediate common stop. No confirmation by design.
        Each arm's Drag Teach subprocess receives SIGINT, which calls
        rm_stop_drag_teach() in the runner.
        """
        self.dual_drag_stop_btn.configure(state="disabled")

        # The popup is displayed only after BOTH controllers have completed
        # rm_stop_drag_teach() and their recorded trajectories are ready.
        self.dual_drag_save_pending = True

        self._stop_panel_drag_from_common(self.left_panel)
        self._stop_panel_drag_from_common(self.right_panel)

        # Keep the common START disabled until both sessions have actually ended.
        if self.dual_drag_active:
            self._monitor_both_drag()


    def _dual_drag_default_save_paths(self):
        from datetime import datetime

        folder = Path.home() / "Downloads" / "RealMan_DragTeach"
        try:
            folder.mkdir(parents=True, exist_ok=True)
        except OSError:
            folder = Path.home()

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return (
            folder / f"left_drag_{stamp}.txt",
            folder / f"right_drag_{stamp}.txt",
        )

    def _browse_dual_drag_save_path(self, target_var, side_label):
        current = target_var.get().strip()
        if current:
            current_path = Path(current).expanduser()
            initial_dir = (
                current_path.parent
                if current_path.parent.exists()
                else Path.home()
            )
            initial_name = current_path.name or f"{side_label.lower()}_drag.txt"
        else:
            left_default, right_default = self._dual_drag_default_save_paths()
            default = left_default if side_label == "LEFT" else right_default
            initial_dir = default.parent
            initial_name = default.name

        path = filedialog.asksaveasfilename(
            title=f"Save {side_label} Drag Teach trajectory",
            initialdir=str(initial_dir),
            initialfile=initial_name,
            defaultextension=".txt",
            filetypes=[
                ("Trajectory text", "*.txt"),
                ("All files", "*.*"),
            ],
            parent=self.dual_drag_save_dialog or self,
        )
        if path:
            target_var.set(path)

    def _show_dual_drag_save_popup(self):
        """
        One common popup for LEFT and RIGHT Drag Teach recordings.
        The user can change either filename, then save both controller
        recordings with one button.
        """
        if self.dual_drag_save_active:
            return

        if (
            self.dual_drag_save_dialog is not None
            and self.dual_drag_save_dialog.winfo_exists()
        ):
            self.dual_drag_save_dialog.lift()
            self.dual_drag_save_dialog.focus_force()
            return

        left_default, right_default = self._dual_drag_default_save_paths()
        self.dual_drag_left_save_var.set(str(left_default))
        self.dual_drag_right_save_var.set(str(right_default))

        dialog = tk.Toplevel(self)
        self.dual_drag_save_dialog = dialog
        dialog.title("Save BOTH Drag Teach Trajectories")
        dialog.configure(bg=BG)
        dialog.transient(self)
        dialog.resizable(True, False)

        try:
            dialog.grab_set()
        except tk.TclError:
            pass

        body = ttk.Frame(
            dialog,
            padding=14,
            style="App.TFrame",
        )
        body.pack(fill="both", expand=True)

        ttk.Label(
            body,
            text="DRAG TEACH STOPPED // SAVE BOTH RECORDINGS",
            style="Kicker.TLabel",
        ).grid(
            row=0,
            column=0,
            columnspan=3,
            sticky="w",
            pady=(0, 5),
        )

        ttk.Label(
            body,
            text=(
                "Choose the two local .txt filenames. "
                "Both recordings will be read from their own controllers."
            ),
            style="Muted.TLabel",
        ).grid(
            row=1,
            column=0,
            columnspan=3,
            sticky="w",
            pady=(0, 12),
        )

        ttk.Label(
            body,
            text="LEFT arm",
            style="TLabel",
        ).grid(row=2, column=0, sticky="w", padx=(0, 8), pady=5)

        left_entry = self._tech_entry(
            body,
            textvariable=self.dual_drag_left_save_var,
            width=66,
        )
        left_entry.grid(
            row=2,
            column=1,
            sticky="ew",
            pady=5,
            ipady=4,
        )

        ttk.Button(
            body,
            text="Browse",
            command=lambda: self._browse_dual_drag_save_path(
                self.dual_drag_left_save_var,
                "LEFT",
            ),
        ).grid(row=2, column=2, padx=(8, 0), pady=5)

        ttk.Label(
            body,
            text="RIGHT arm",
            style="TLabel",
        ).grid(row=3, column=0, sticky="w", padx=(0, 8), pady=5)

        right_entry = self._tech_entry(
            body,
            textvariable=self.dual_drag_right_save_var,
            width=66,
        )
        right_entry.grid(
            row=3,
            column=1,
            sticky="ew",
            pady=5,
            ipady=4,
        )

        ttk.Button(
            body,
            text="Browse",
            command=lambda: self._browse_dual_drag_save_path(
                self.dual_drag_right_save_var,
                "RIGHT",
            ),
        ).grid(row=3, column=2, padx=(8, 0), pady=5)

        ttk.Label(
            body,
            text=(
                "The LEFT and RIGHT files are independent. "
                "SAVE BOTH does not move either robot."
            ),
            style="Muted.TLabel",
        ).grid(
            row=4,
            column=0,
            columnspan=3,
            sticky="w",
            pady=(8, 12),
        )

        buttons = ttk.Frame(body, style="App.TFrame")
        buttons.grid(
            row=5,
            column=0,
            columnspan=3,
            sticky="e",
        )

        ttk.Button(
            buttons,
            text="CANCEL",
            command=self._close_dual_drag_save_popup,
        ).pack(side="left", padx=(0, 8))

        self.dual_drag_save_both_btn = ttk.Button(
            buttons,
            text="💾 SAVE BOTH .TXT",
            command=self._save_both_drag_files,
            style="Run.TButton",
        )
        self.dual_drag_save_both_btn.pack(side="left")

        body.columnconfigure(1, weight=1)

        dialog.protocol(
            "WM_DELETE_WINDOW",
            self._close_dual_drag_save_popup,
        )
        dialog.update_idletasks()

        width = max(820, dialog.winfo_reqwidth())
        height = max(260, dialog.winfo_reqheight())
        dialog.geometry(f"{width}x{height}")

        dialog.lift()
        left_entry.focus_set()

    def _close_dual_drag_save_popup(self):
        dialog = self.dual_drag_save_dialog
        self.dual_drag_save_dialog = None

        if dialog is not None:
            try:
                dialog.grab_release()
            except tk.TclError:
                pass
            try:
                dialog.destroy()
            except tk.TclError:
                pass

    def _save_both_drag_files(self):
        if self.dual_drag_save_active:
            return

        left_path_text = self.dual_drag_left_save_var.get().strip()
        right_path_text = self.dual_drag_right_save_var.get().strip()

        if not left_path_text or not right_path_text:
            messagebox.showerror(
                "Save Both Drag Trajectories",
                "Please provide both LEFT and RIGHT .txt file paths.",
                parent=self.dual_drag_save_dialog or self,
            )
            return

        left_path = Path(left_path_text).expanduser()
        right_path = Path(right_path_text).expanduser()

        if left_path.resolve() == right_path.resolve():
            messagebox.showerror(
                "Save Both Drag Trajectories",
                "LEFT and RIGHT must use different .txt filenames.",
                parent=self.dual_drag_save_dialog or self,
            )
            return

        if left_path.suffix.lower() != ".txt":
            left_path = left_path.with_suffix(".txt")
            self.dual_drag_left_save_var.set(str(left_path))

        if right_path.suffix.lower() != ".txt":
            right_path = right_path.with_suffix(".txt")
            self.dual_drag_right_save_var.set(str(right_path))

        try:
            left_path.parent.mkdir(parents=True, exist_ok=True)
            right_path.parent.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            messagebox.showerror(
                "Save Both Drag Trajectories",
                f"Could not create the save folder:\n\n{exc}",
                parent=self.dual_drag_save_dialog or self,
            )
            return

        if self.left_panel.has_active_command() or self.right_panel.has_active_command():
            messagebox.showwarning(
                "Controllers busy",
                "Wait until both Drag Teach stop commands have completely finished.",
                parent=self.dual_drag_save_dialog or self,
            )
            return

        left_env = self.left_panel._validate_environment()
        if not left_env:
            return

        right_env = self.right_panel._validate_environment()
        if not right_env:
            return

        left_rm, left_sdk = left_env
        right_rm, right_sdk = right_env

        left_cmd = self.left_panel._base_command(left_rm, left_sdk) + [
            "--drag-save",
            str(left_path),
            "--side",
            "left",
            "--yes",
        ]
        right_cmd = self.right_panel._base_command(right_rm, right_sdk) + [
            "--drag-save",
            str(right_path),
            "--side",
            "right",
            "--yes",
        ]

        # Keep the same paths visible later in each arm's individual Drag tab.
        self.left_panel.drag_save_var.set(str(left_path))
        self.right_panel.drag_save_var.set(str(right_path))

        self.dual_drag_save_active = True

        if hasattr(self, "dual_drag_save_both_btn"):
            try:
                self.dual_drag_save_both_btn.configure(state="disabled")
            except tk.TclError:
                pass

        self._close_dual_drag_save_popup()

        try:
            self.left_panel._start_process(
                left_cmd,
                "DRAG SAVE // Saving LEFT trajectory",
                motion=False,
            )
            self.right_panel._start_process(
                right_cmd,
                "DRAG SAVE // Saving RIGHT trajectory",
                motion=False,
            )
        except Exception as exc:
            self.dual_drag_save_active = False
            messagebox.showerror(
                "Save Both Drag Trajectories",
                f"Could not start both save operations:\n\n{exc}",
                parent=self,
            )
            return

        self.dual_drag_left_saved_path = str(left_path)
        self.dual_drag_right_saved_path = str(right_path)

        # Make the latest saved pair the default pair for RUN BOTH .TXT.
        self.dual_file_left_var.set(str(left_path))
        self.dual_file_right_var.set(str(right_path))

        self.dual_drag_save_after_id = self.after(
            150,
            self._monitor_dual_drag_saves,
        )

    def _monitor_dual_drag_saves(self):
        if not self.dual_drag_save_active:
            return

        if (
            self.left_panel.has_active_command()
            or self.right_panel.has_active_command()
        ):
            self.dual_drag_save_after_id = self.after(
                150,
                self._monitor_dual_drag_saves,
            )
            return

        self.dual_drag_save_active = False
        self.dual_drag_save_after_id = None

        left_status = self.left_panel.status_var.get()
        right_status = self.right_panel.status_var.get()

        left_ok = left_status.startswith("DRAG SAVE COMPLETE")
        right_ok = right_status.startswith("DRAG SAVE COMPLETE")

        if left_ok and right_ok:
            messagebox.showinfo(
                "Drag Teach Saved",
                "Both Drag Teach trajectories were saved successfully.\n\n"
                f"LEFT:\n{self.dual_drag_left_saved_path}\n\n"
                f"RIGHT:\n{self.dual_drag_right_saved_path}",
                parent=self,
            )
        else:
            problems = []
            if not left_ok:
                problems.append(f"LEFT: {left_status}")
            if not right_ok:
                problems.append(f"RIGHT: {right_status}")

            messagebox.showerror(
                "Drag Teach Save",
                "One or both trajectory files were not confirmed as saved.\n\n"
                + "\n".join(problems),
                parent=self,
            )


    def _browse_dual_file_run_path(self, target_var, side_label):
        current = target_var.get().strip()
        initial_dir = Path.home() / "Downloads"
        initial_name = ""

        if current:
            p = Path(current).expanduser()
            if p.parent.exists():
                initial_dir = p.parent
            initial_name = p.name

        path = filedialog.askopenfilename(
            title=f"Choose {side_label} Drag Teach .txt",
            initialdir=str(initial_dir),
            initialfile=initial_name,
            filetypes=[
                ("Trajectory text", "*.txt"),
                ("All files", "*.*"),
            ],
            parent=self.dual_file_run_dialog or self,
        )
        if path:
            target_var.set(path)

    def _show_dual_file_run_popup(self):
        if self.dual_file_run_active:
            messagebox.showwarning(
                "RUN BOTH .TXT",
                "A dual file run is already active.",
                parent=self,
            )
            return

        if self.dual_drag_active:
            messagebox.showwarning(
                "Dual Drag active",
                "Stop both Drag Teach sessions before running saved files.",
                parent=self,
            )
            return

        if self.dual_run_active:
            messagebox.showwarning(
                "Dual Run active",
                "Wait until the current Dual Run is complete.",
                parent=self,
            )
            return

        if self.left_panel.has_active_command() or self.right_panel.has_active_command():
            messagebox.showwarning(
                "Arm busy",
                "Both arms must be idle before RUN BOTH .TXT.",
                parent=self,
            )
            return

        if (
            self.dual_file_run_dialog is not None
            and self.dual_file_run_dialog.winfo_exists()
        ):
            self.dual_file_run_dialog.lift()
            self.dual_file_run_dialog.focus_force()
            return

        # Prefer the last SAVE BOTH pair when available.
        if not self.dual_file_left_var.get().strip():
            last_left = getattr(self, "dual_drag_left_saved_path", "")
            if last_left:
                self.dual_file_left_var.set(last_left)

        if not self.dual_file_right_var.get().strip():
            last_right = getattr(self, "dual_drag_right_saved_path", "")
            if last_right:
                self.dual_file_right_var.set(last_right)

        dialog = tk.Toplevel(self)
        self.dual_file_run_dialog = dialog
        dialog.title("RUN BOTH Drag Teach .TXT Files")
        dialog.configure(bg=BG)
        dialog.transient(self)
        dialog.resizable(True, False)

        try:
            dialog.grab_set()
        except tk.TclError:
            pass

        body = ttk.Frame(dialog, padding=14, style="App.TFrame")
        body.pack(fill="both", expand=True)

        ttk.Label(
            body,
            text="RUN BOTH SAVED DRAG TRAJECTORIES",
            style="Kicker.TLabel",
        ).grid(
            row=0,
            column=0,
            columnspan=4,
            sticky="w",
            pady=(0, 5),
        )

        ttk.Label(
            body,
            text=(
                "Both files are uploaded first. Neither arm moves until BOTH "
                "uploads are verified, then both Program IDs are started together."
            ),
            style="Muted.TLabel",
        ).grid(
            row=1,
            column=0,
            columnspan=4,
            sticky="w",
            pady=(0, 12),
        )

        ttk.Label(body, text="LEFT .txt", style="TLabel").grid(
            row=2, column=0, sticky="w", padx=(0, 8), pady=5
        )
        left_entry = self._tech_entry(
            body,
            textvariable=self.dual_file_left_var,
            width=58,
        )
        left_entry.grid(row=2, column=1, sticky="ew", pady=5, ipady=4)

        ttk.Button(
            body,
            text="Browse",
            command=lambda: self._browse_dual_file_run_path(
                self.dual_file_left_var,
                "LEFT",
            ),
        ).grid(row=2, column=2, padx=(8, 12), pady=5)

        ttk.Label(body, text="Program ID", style="TLabel").grid(
            row=2, column=3, sticky="e", padx=(0, 5), pady=5
        )
        left_slot = tk.Spinbox(
            body,
            from_=1,
            to=100,
            textvariable=self.dual_file_left_slot_var,
            width=5,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="white",
            buttonbackground=PANEL_2,
            relief="flat",
        )
        left_slot.grid(row=2, column=4, sticky="w", pady=5)

        ttk.Label(body, text="RIGHT .txt", style="TLabel").grid(
            row=3, column=0, sticky="w", padx=(0, 8), pady=5
        )
        right_entry = self._tech_entry(
            body,
            textvariable=self.dual_file_right_var,
            width=58,
        )
        right_entry.grid(row=3, column=1, sticky="ew", pady=5, ipady=4)

        ttk.Button(
            body,
            text="Browse",
            command=lambda: self._browse_dual_file_run_path(
                self.dual_file_right_var,
                "RIGHT",
            ),
        ).grid(row=3, column=2, padx=(8, 12), pady=5)

        ttk.Label(body, text="Program ID", style="TLabel").grid(
            row=3, column=3, sticky="e", padx=(0, 5), pady=5
        )
        right_slot = tk.Spinbox(
            body,
            from_=1,
            to=100,
            textvariable=self.dual_file_right_slot_var,
            width=5,
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="white",
            buttonbackground=PANEL_2,
            relief="flat",
        )
        right_slot.grid(row=3, column=4, sticky="w", pady=5)

        ttk.Label(
            body,
            text=(
                "Important: the selected Program ID on each controller will be "
                "replaced by the uploaded .txt file."
            ),
            style="PanelMuted.TLabel",
        ).grid(
            row=4,
            column=0,
            columnspan=5,
            sticky="w",
            pady=(8, 12),
        )

        buttons = ttk.Frame(body, style="App.TFrame")
        buttons.grid(
            row=5,
            column=0,
            columnspan=5,
            sticky="e",
        )

        ttk.Button(
            buttons,
            text="CANCEL",
            command=self._close_dual_file_run_popup,
        ).pack(side="left", padx=(0, 8))

        ttk.Button(
            buttons,
            text="▶▶ UPLOAD + RUN BOTH",
            command=self._run_both_txt_files,
            style="Run.TButton",
        ).pack(side="left")

        body.columnconfigure(1, weight=1)

        dialog.protocol(
            "WM_DELETE_WINDOW",
            self._close_dual_file_run_popup,
        )
        dialog.update_idletasks()

        width = max(900, dialog.winfo_reqwidth())
        height = max(285, dialog.winfo_reqheight())
        dialog.geometry(f"{width}x{height}")

        dialog.lift()
        left_entry.focus_set()

    def _close_dual_file_run_popup(self):
        dialog = self.dual_file_run_dialog
        self.dual_file_run_dialog = None

        if dialog is not None:
            try:
                dialog.grab_release()
            except tk.TclError:
                pass
            try:
                dialog.destroy()
            except tk.TclError:
                pass

    def _run_both_txt_files(self):
        if self.dual_file_run_active:
            return

        left_path = Path(
            self.dual_file_left_var.get().strip()
        ).expanduser()
        right_path = Path(
            self.dual_file_right_var.get().strip()
        ).expanduser()

        if not left_path.is_file():
            messagebox.showerror(
                "RUN BOTH .TXT",
                f"LEFT trajectory file does not exist:\n\n{left_path}",
                parent=self.dual_file_run_dialog or self,
            )
            return

        if not right_path.is_file():
            messagebox.showerror(
                "RUN BOTH .TXT",
                f"RIGHT trajectory file does not exist:\n\n{right_path}",
                parent=self.dual_file_run_dialog or self,
            )
            return

        try:
            left_slot = int(self.dual_file_left_slot_var.get().strip())
            right_slot = int(self.dual_file_right_slot_var.get().strip())
        except Exception:
            messagebox.showerror(
                "RUN BOTH .TXT",
                "Program IDs must be numbers from 1 to 100.",
                parent=self.dual_file_run_dialog or self,
            )
            return

        if not 1 <= left_slot <= 100 or not 1 <= right_slot <= 100:
            messagebox.showerror(
                "RUN BOTH .TXT",
                "LEFT and RIGHT Program IDs must be from 1 to 100.",
                parent=self.dual_file_run_dialog or self,
            )
            return

        if self.left_panel.has_active_command() or self.right_panel.has_active_command():
            messagebox.showwarning(
                "Arm busy",
                "Both arms must be idle before starting.",
                parent=self.dual_file_run_dialog or self,
            )
            return

        left_env = self.left_panel._validate_environment()
        if not left_env:
            return
        right_env = self.right_panel._validate_environment()
        if not right_env:
            return

        ok = messagebox.askyesno(
            "Confirm RUN BOTH .TXT",
            "BOTH ROBOT ARMS WILL MOVE.\n\n"
            f"LEFT\n{left_path.name}\nProgram ID {left_slot}\n\n"
            f"RIGHT\n{right_path.name}\nProgram ID {right_slot}\n\n"
            "First both .txt files will be uploaded and verified. "
            "Then both programs will start together.\n\n"
            "The selected Program IDs will be replaced.\n\n"
            "Confirm both work areas are clear and both hardware E-stops "
            "are accessible.\n\n"
            "UPLOAD + RUN BOTH?",
            icon="warning",
            parent=self.dual_file_run_dialog or self,
        )
        if not ok:
            return

        left_rm, left_sdk = left_env
        right_rm, right_sdk = right_env

        # Upload only. Movement starts later, after both uploads are verified.
        left_upload_cmd = self.left_panel._base_command(left_rm, left_sdk) + [
            str(left_path),
            str(left_slot),
            "--side",
            "left",
            "--yes",
        ]
        right_upload_cmd = self.right_panel._base_command(right_rm, right_sdk) + [
            str(right_path),
            str(right_slot),
            "--side",
            "right",
            "--yes",
        ]

        self._close_dual_file_run_popup()

        self.dual_file_run_active = True
        self.dual_file_left_slot = left_slot
        self.dual_file_right_slot = right_slot
        self.dual_file_left_path = str(left_path)
        self.dual_file_right_path = str(right_path)

        self.dual_file_run_btn.configure(state="disabled")
        self.dual_drag_start_btn.configure(state="disabled")
        self.dual_run_btn.configure(state="disabled")
        self.dual_sync_btn.configure(state="disabled")
        self.dual_stop_btn.configure(state="disabled")

        self.dual_status_var.set(
            f"TXT UPLOAD // LEFT {left_slot} + RIGHT {right_slot}"
        )
        self.dual_runtime_var.set("0.0 s")
        self.dual_run_started_at = __import__("time").monotonic()

        try:
            self.left_panel._start_process(
                left_upload_cmd,
                f"DUAL TXT UPLOAD // LEFT Program {left_slot}",
                motion=False,
            )
            self.right_panel._start_process(
                right_upload_cmd,
                f"DUAL TXT UPLOAD // RIGHT Program {right_slot}",
                motion=False,
            )
        except Exception as exc:
            self._finish_dual_file_run_fault(
                f"Could not start uploads: {exc}"
            )
            return

        self.dual_file_run_after_id = self.after(
            150,
            self._wait_for_dual_txt_uploads,
        )

    def _wait_for_dual_txt_uploads(self):
        if not self.dual_file_run_active:
            return

        now = __import__("time").monotonic()
        if self.dual_run_started_at is not None:
            self.dual_runtime_var.set(
                f"{now - self.dual_run_started_at:.1f} s"
            )

        left_active = self.left_panel.has_active_command()
        right_active = self.right_panel.has_active_command()

        if left_active or right_active:
            bits = [
                "L UPLOADING" if left_active else "L READY",
                "R UPLOADING" if right_active else "R READY",
            ]
            self.dual_status_var.set(
                "TXT // " + " // ".join(bits)
            )
            self.dual_file_run_after_id = self.after(
                150,
                self._wait_for_dual_txt_uploads,
            )
            return

        left_status = self.left_panel.status_var.get()
        right_status = self.right_panel.status_var.get()

        # Standard track upload completion uses VERIFIED/COMPLETE in this UI.
        left_ok = (
            left_status.startswith("VERIFIED //")
            or left_status.startswith("COMPLETE //")
        )
        right_ok = (
            right_status.startswith("VERIFIED //")
            or right_status.startswith("COMPLETE //")
        )

        if not left_ok or not right_ok:
            problems = []
            if not left_ok:
                problems.append(f"LEFT: {left_status}")
            if not right_ok:
                problems.append(f"RIGHT: {right_status}")

            self._finish_dual_file_run_fault(
                "One or both uploads failed.\n\n" + "\n".join(problems)
            )
            return

        self._start_uploaded_txt_together()

    def _start_uploaded_txt_together(self):
        left_env = self.left_panel._validate_environment()
        right_env = self.right_panel._validate_environment()
        if not left_env or not right_env:
            self._finish_dual_file_run_fault(
                "Runner/SDK validation failed before synchronized start."
            )
            return

        left_rm, left_sdk = left_env
        right_rm, right_sdk = right_env

        left_slot = self.dual_file_left_slot
        right_slot = self.dual_file_right_slot

        left_cmd = self.left_panel._base_command(left_rm, left_sdk) + [
            "--run",
            str(left_slot),
            "--side",
            "left",
            "--yes",
        ]
        right_cmd = self.right_panel._base_command(right_rm, right_sdk) + [
            "--run",
            str(right_slot),
            "--side",
            "right",
            "--yes",
        ]

        launch_barrier = threading.Barrier(2)

        self.dual_status_var.set(
            f"TXT VERIFIED // STARTING LEFT {left_slot} + RIGHT {right_slot}"
        )
        self.dual_stop_btn.configure(state="normal")

        try:
            self.left_panel._start_process_synced(
                left_cmd,
                f"DUAL TXT RUN // Program {left_slot}",
                launch_barrier,
            )
            self.right_panel._start_process_synced(
                right_cmd,
                f"DUAL TXT RUN // Program {right_slot}",
                launch_barrier,
            )
        except Exception as exc:
            self._finish_dual_file_run_fault(
                f"Could not start synchronized programs: {exc}"
            )
            self.left_panel._slow_stop()
            self.right_panel._slow_stop()
            return

        self._monitor_dual_file_run_motion()

    def _monitor_dual_file_run_motion(self):
        if not self.dual_file_run_active:
            return

        now = __import__("time").monotonic()
        if self.dual_run_started_at is not None:
            self.dual_runtime_var.set(
                f"{now - self.dual_run_started_at:.1f} s"
            )

        left_active = self.left_panel.has_active_command()
        right_active = self.right_panel.has_active_command()

        if left_active or right_active:
            bits = [
                "LEFT ACTIVE" if left_active else "LEFT COMPLETE",
                "RIGHT ACTIVE" if right_active else "RIGHT COMPLETE",
            ]
            self.dual_status_var.set(
                "TXT RUN // " + " // ".join(bits)
            )
            self.dual_file_run_after_id = self.after(
                100,
                self._monitor_dual_file_run_motion,
            )
            return

        elapsed = (
            now - self.dual_run_started_at
            if self.dual_run_started_at is not None
            else 0.0
        )
        self.dual_runtime_var.set(f"{elapsed:.1f} s")

        left_status = self.left_panel.status_var.get()
        right_status = self.right_panel.status_var.get()

        if (
            left_status.startswith("COMPLETE //")
            and right_status.startswith("COMPLETE //")
        ):
            self.dual_status_var.set(
                f"TXT RUN COMPLETE // Both arms finished // {elapsed:.1f} s"
            )
        else:
            self.dual_status_var.set(
                "TXT RUN FINISHED // Check LEFT and RIGHT telemetry"
            )

        self._finish_dual_file_run_state()

    def _finish_dual_file_run_fault(self, message):
        self.dual_status_var.set("TXT FAULT // " + message.replace("\n", " "))
        self._finish_dual_file_run_state()

        messagebox.showerror(
            "RUN BOTH .TXT",
            message,
            parent=self,
        )

    def _finish_dual_file_run_state(self):
        self.dual_file_run_active = False
        self.dual_file_run_after_id = None
        self.dual_run_started_at = None

        self.dual_file_run_btn.configure(state="normal")
        self.dual_drag_start_btn.configure(state="normal")
        self.dual_run_btn.configure(state="normal")
        self.dual_sync_btn.configure(state="normal")
        self.dual_stop_btn.configure(state="disabled")


    def _build_sequence_page(self, parent):
        root = ttk.Frame(
            parent,
            padding=12,
            style="App.TFrame",
        )
        root.pack(fill="both", expand=True)

        header = ttk.Frame(root, style="App.TFrame")
        header.pack(fill="x", pady=(0, 7))

        header_left = ttk.Frame(header, style="App.TFrame")
        header_left.pack(side="left", fill="x", expand=True)

        ttk.Label(
            header_left,
            text="PAGE 02 // DUAL ARM SEQUENCE BUILDER // SAFE START + LIVE STATUS",
            style="Kicker.TLabel",
        ).pack(anchor="w")

        ttk.Label(
            header_left,
            text="Build independent LEFT and RIGHT cleaning workflows",
            style="Title.TLabel",
        ).pack(anchor="w", pady=(2, 1))

        ttk.Label(
            header_left,
            text=(
                "Mix stored Programs, Pole, 6-channel Hand, Wait, and "
                "Global Points (MoveJ / MoveL + speed) in any order."
            ),
            style="Muted.TLabel",
        ).pack(anchor="w")

        refresh_bar = ttk.Frame(header, style="App.TFrame")
        refresh_bar.pack(side="right", anchor="ne")

        ttk.Button(
            refresh_bar,
            text="↻ PROGRAMS",
            command=self._page2_refresh_programs,
            style="CompactBlue.TButton",
        ).pack(side="left", padx=(0, 5))

        ttk.Button(
            refresh_bar,
            text="↻ GLOBAL POINTS",
            command=self._page2_refresh_globals,
            style="CompactBlue.TButton",
        ).pack(side="left")

        dual = ttk.LabelFrame(
            root,
            text="DUAL SEQUENCE CONTROL",
            padding=(8, 5),
            style="Tech.TLabelframe",
        )
        dual.pack(fill="x", pady=(0, 8))

        self.page2_run_both_btn = ttk.Button(
            dual,
            text="▶▶ RUN BOTH // SAFE TIMELINE",
            command=self._run_both_page2_sequences,
            style="Run.TButton",
        )
        self.page2_run_both_btn.pack(side="left", padx=(0, 6))

        self.page2_stop_both_btn = ttk.Button(
            dual,
            text="■ STOP BOTH SEQUENCES",
            command=self._stop_both_page2_sequences,
            state="disabled",
            style="Stop.TButton",
        )
        self.page2_stop_both_btn.pack(side="left", padx=(0, 10))

        self.page2_alt_confirm_check = ttk.Checkbutton(
            dual,
            text="CONFIRM BEFORE WAITING ARM",
            variable=self.page2_alt_confirm_each_var,
        )
        self.page2_alt_confirm_check.pack(
            side="left",
            padx=(0, 10),
        )

        ttk.Label(
            dual,
            text="OVERALL SPEED %",
            style="Panel.TLabel",
        ).pack(side="left", padx=(0, 5))

        self.page2_overall_speed_entry = tk.Entry(
            dual,
            textvariable=self.page2_overall_speed_var,
            width=5,
            justify="center",
            bg=ENTRY_BG,
            fg=TEXT,
            insertbackground="#FFFFFF",
            insertwidth=3,
            selectbackground=BLUE,
            selectforeground="#FFFFFF",
            relief="flat",
            highlightthickness=1,
            highlightbackground=BORDER,
            highlightcolor=CYAN,
            font=("DejaVu Sans", 10, "bold"),
        )
        self.page2_overall_speed_entry.pack(
            side="left",
            padx=(0, 10),
            ipady=3,
        )

        ttk.Label(
            dual,
            textvariable=self.page2_status_var,
            style="PanelMuted.TLabel",
        ).pack(side="left", fill="x", expand=True)

        ttk.Label(
            dual,
            textvariable=self.page2_runtime_var,
            style="RuntimeValue.TLabel",
        ).pack(side="right")

        live = ttk.LabelFrame(
            root,
            text="LIVE ARM STATUS",
            padding=(8, 5),
            style="Tech.TLabelframe",
        )
        live.pack(fill="x", pady=(0, 6))
        live.columnconfigure(0, weight=1, uniform="livearm")
        live.columnconfigure(1, weight=1, uniform="livearm")

        ttk.Label(
            live,
            textvariable=self.page2_left_live_var,
            style="StatusValue.TLabel",
            anchor="w",
        ).grid(row=0, column=0, sticky="ew", padx=(0, 8))

        ttk.Label(
            live,
            textvariable=self.page2_right_live_var,
            style="StatusValue.TLabel",
            anchor="w",
        ).grid(row=0, column=1, sticky="ew", padx=(8, 0))

        split = ttk.Frame(root, style="App.TFrame")
        split.pack(fill="both", expand=True)
        split.columnconfigure(0, weight=1, uniform="page2arms")
        split.columnconfigure(1, weight=1, uniform="page2arms")
        split.rowconfigure(0, weight=1)

        left_holder = ttk.Frame(split, style="App.TFrame")
        right_holder = ttk.Frame(split, style="App.TFrame")
        left_holder.grid(
            row=0, column=0, sticky="nsew", padx=(0, 5)
        )
        right_holder.grid(
            row=0, column=1, sticky="nsew", padx=(5, 0)
        )

        self.left_sequence_panel = WorkflowSequencePanel(
            left_holder,
            self,
            self.left_panel,
            "left",
        )
        self.left_sequence_panel.pack(fill="both", expand=True)

        self.right_sequence_panel = WorkflowSequencePanel(
            right_holder,
            self,
            self.right_panel,
            "right",
        )
        self.right_sequence_panel.pack(fill="both", expand=True)

    def _page2_refresh_sources(self):
        if hasattr(self, "left_sequence_panel"):
            self.left_sequence_panel.refresh_sources()
        if hasattr(self, "right_sequence_panel"):
            self.right_sequence_panel.refresh_sources()

    def _page2_refresh_programs(self):
        if (
            self.left_panel.has_active_command()
            or self.right_panel.has_active_command()
            or self.page2_dual_active
        ):
            messagebox.showwarning(
                "Controllers busy",
                "Both controllers must be idle before refreshing programs.",
                parent=self,
            )
            return

        self.page2_status_var.set(
            "PAGE 02 // Reading stored programs from both controllers"
        )
        self.left_panel._list_slots()
        self.right_panel._list_slots()
        self.after(200, self._page2_wait_program_refresh)

    def _page2_wait_program_refresh(self):
        if (
            self.left_panel.has_active_command()
            or self.right_panel.has_active_command()
        ):
            self.after(200, self._page2_wait_program_refresh)
            return

        self._page2_refresh_sources()
        self.page2_status_var.set(
            "PAGE 02 // Program lists refreshed"
        )

    def _page2_refresh_globals(self):
        if (
            self.left_panel.has_active_command()
            or self.right_panel.has_active_command()
            or self.page2_dual_active
        ):
            messagebox.showwarning(
                "Controllers busy",
                "Both controllers must be idle before refreshing global points.",
                parent=self,
            )
            return

        self.page2_status_var.set(
            "PAGE 02 // Reading global points from both controllers"
        )
        self.left_panel._refresh_global_points()
        self.right_panel._refresh_global_points()
        self.after(200, self._page2_wait_global_refresh)

    def _page2_wait_global_refresh(self):
        if (
            self.left_panel.has_active_command()
            or self.right_panel.has_active_command()
        ):
            self.after(200, self._page2_wait_global_refresh)
            return

        self._page2_refresh_sources()
        self.page2_status_var.set(
            "PAGE 02 // Global point lists refreshed"
        )

    def _page2_builder(self, side):
        return (
            self.left_sequence_panel
            if side == "left"
            else self.right_sequence_panel
        )

    def _page2_arm_panel(self, side):
        return self.left_panel if side == "left" else self.right_panel

    def _page2_overall_speed(self):
        try:
            value = int(float(self.page2_overall_speed_var.get().strip()))
        except Exception:
            value = 100

        value = max(1, min(100, value))
        self.page2_overall_speed_var.set(str(value))
        return value

    def _page2_sequence_command(self, side):
        builder = self._page2_builder(side)
        arm_panel = self._page2_arm_panel(side)

        sequence_path = builder.write_temp()
        env = arm_panel._validate_environment()
        if not env:
            return None

        rm_upload, sdk = env
        overall_speed = self._page2_overall_speed()

        return arm_panel._base_command(rm_upload, sdk) + [
            "--workflow-sequence-file",
            str(sequence_path),
            "--workflow-overall-speed",
            str(overall_speed),
            "--side",
            side,
            "--yes",
        ]

    def _run_single_page2_sequence(self, side):
        if self.page2_dual_active:
            messagebox.showwarning(
                "Dual sequence active",
                "Wait for RUN BOTH SEQUENCES to finish first.",
                parent=self,
            )
            return

        builder = self._page2_builder(side)
        arm_panel = self._page2_arm_panel(side)

        if not builder.steps:
            messagebox.showinfo(
                "Sequence",
                f"{side.upper()} sequence is empty.",
                parent=self,
            )
            return

        if arm_panel.has_active_command():
            messagebox.showwarning(
                "Arm busy",
                f"{side.upper()} arm already has an active command.",
                parent=self,
            )
            return

        try:
            cmd = self._page2_sequence_command(side)
        except Exception as exc:
            messagebox.showerror(
                "Sequence",
                f"Could not prepare {side.upper()} sequence:\n\n{exc}",
                parent=self,
            )
            return

        if not cmd:
            return

        ok = messagebox.askyesno(
            f"Run {side.upper()} Sequence",
            f"{side.upper()} ROBOT ARM WILL MOVE.\n\n"
            f"Sequence steps: {len(builder.steps)}\n"
            f"Overall controller speed: {self._page2_overall_speed()}%\n\n"
            "Confirm the work area is clear and the hardware E-stop "
            "is accessible.\n\n"
            f"RUN {side.upper()} SEQUENCE?",
            icon="warning",
            parent=self,
        )
        if not ok:
            return

        builder.set_running(True)

        try:
            arm_panel._start_process(
                cmd,
                f"PAGE 02 // {side.upper()} sequence starting",
                motion=True,
            )
        except Exception as exc:
            builder.set_running(False)
            messagebox.showerror(
                "Sequence",
                f"Could not start {side.upper()} sequence:\n\n{exc}",
                parent=self,
            )
            return

        self.after(
            200,
            lambda s=side: self._monitor_single_page2_sequence(s),
        )

    def _monitor_single_page2_sequence(self, side):
        arm_panel = self._page2_arm_panel(side)
        builder = self._page2_builder(side)

        if arm_panel.has_active_command():
            self.after(
                200,
                lambda s=side: self._monitor_single_page2_sequence(s),
            )
            return

        builder.set_running(False)
        builder.status_var.set(
            f"{side.upper()} // {arm_panel.status_var.get()}"
        )

    def _stop_single_page2_sequence(self, side):
        arm_panel = self._page2_arm_panel(side)
        builder = self._page2_builder(side)

        builder.status_var.set(
            f"{side.upper()} // STOP REQUESTED"
        )
        arm_panel._slow_stop()

    def _page2_execution_mode(self):
        # v47: Page 02 RUN BOTH always uses per-Program Dual Rules.
        return "PROGRAM_RULES"

    def _start_page2_serial_first(
        self,
        first_side,
        second_side,
        first_cmd,
        second_cmd,
    ):
        first_panel = self._page2_arm_panel(first_side)
        first_builder = self._page2_builder(first_side)
        second_builder = self._page2_builder(second_side)

        self.page2_serial_first_side = first_side
        self.page2_serial_second_side = second_side
        self.page2_serial_second_cmd = second_cmd

        first_builder.set_running(True)
        second_builder.set_running(False)

        self.page2_stop_both_btn.configure(state="normal")
        self.page2_status_var.set(
            f"PAGE 02 ONE-ARM MODE // {first_side.upper()} ACTIVE // "
            f"{second_side.upper()} WAITING"
        )

        try:
            first_panel._start_process(
                first_cmd,
                f"PAGE 02 ONE-ARM MODE // {first_side.upper()} FIRST",
                motion=True,
            )
        except Exception as exc:
            self.page2_status_var.set(
                f"PAGE 02 SERIAL FAULT // {exc}"
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_dual_after_id = self.after(
            150,
            self._monitor_page2_serial_first,
        )

    def _monitor_page2_serial_first(self):
        if not self.page2_dual_active:
            return

        now = __import__("time").monotonic()
        if self.page2_dual_started_at is not None:
            self.page2_runtime_var.set(
                f"{now - self.page2_dual_started_at:.1f} s"
            )

        first_side = self.page2_serial_first_side
        second_side = self.page2_serial_second_side
        first_panel = self._page2_arm_panel(first_side)
        first_builder = self._page2_builder(first_side)

        if first_panel.has_active_command():
            self.page2_status_var.set(
                f"PAGE 02 ONE-ARM MODE // {first_side.upper()} ACTIVE // "
                f"{second_side.upper()} WAITING"
            )
            self.page2_dual_after_id = self.after(
                150,
                self._monitor_page2_serial_first,
            )
            return

        first_builder.set_running(False)
        first_status = first_panel.status_var.get()

        if not first_status.startswith("COMPLETE //"):
            self.page2_status_var.set(
                f"PAGE 02 SERIAL STOPPED // {first_side.upper()} NOT COMPLETE // "
                f"{second_side.upper()} NOT STARTED"
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_status_var.set(
            f"PAGE 02 SERIAL // {first_side.upper()} COMPLETE // "
            f"WAITING CONFIRMATION FOR {second_side.upper()}"
        )
        self.page2_stop_both_btn.configure(state="disabled")

        proceed = messagebox.askyesno(
            "Start Second Arm",
            f"{first_side.upper()} SEQUENCE COMPLETED SUCCESSFULLY.\n\n"
            f"{second_side.upper()} arm has NOT started yet.\n\n"
            f"Start {second_side.upper()} sequence now?",
            icon="warning",
            parent=self,
        )

        if not proceed:
            self.page2_status_var.set(
                f"PAGE 02 SERIAL PAUSED // {first_side.upper()} COMPLETE // "
                f"{second_side.upper()} CANCELLED"
            )
            self._finish_page2_dual_sequence()
            return

        self._start_page2_serial_second()

    def _start_page2_serial_second(self):
        second_side = self.page2_serial_second_side
        second_panel = self._page2_arm_panel(second_side)
        second_builder = self._page2_builder(second_side)

        if not self.page2_dual_active:
            return

        if second_panel.has_active_command():
            self.page2_status_var.set(
                f"PAGE 02 SERIAL FAULT // {second_side.upper()} unexpectedly busy"
            )
            self._finish_page2_dual_sequence()
            return

        second_builder.set_running(True)
        self.page2_stop_both_btn.configure(state="normal")
        self.page2_status_var.set(
            f"PAGE 02 ONE-ARM MODE // "
            f"{self.page2_serial_first_side.upper()} COMPLETE // "
            f"{second_side.upper()} ACTIVE"
        )

        try:
            second_panel._start_process(
                self.page2_serial_second_cmd,
                f"PAGE 02 ONE-ARM MODE // {second_side.upper()} SECOND",
                motion=True,
            )
        except Exception as exc:
            self.page2_status_var.set(
                f"PAGE 02 SERIAL FAULT // {exc}"
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_dual_after_id = self.after(
            150,
            self._monitor_page2_serial_second,
        )

    def _monitor_page2_serial_second(self):
        if not self.page2_dual_active:
            return

        now = __import__("time").monotonic()
        if self.page2_dual_started_at is not None:
            self.page2_runtime_var.set(
                f"{now - self.page2_dual_started_at:.1f} s"
            )

        second_side = self.page2_serial_second_side
        second_panel = self._page2_arm_panel(second_side)
        second_builder = self._page2_builder(second_side)

        if second_panel.has_active_command():
            self.page2_dual_after_id = self.after(
                150,
                self._monitor_page2_serial_second,
            )
            return

        second_builder.set_running(False)

        elapsed = (
            now - self.page2_dual_started_at
            if self.page2_dual_started_at is not None
            else 0.0
        )
        self.page2_runtime_var.set(f"{elapsed:.1f} s")

        if second_panel.status_var.get().startswith("COMPLETE //"):
            self.page2_status_var.set(
                f"PAGE 02 SERIAL COMPLETE // "
                f"{self.page2_serial_first_side.upper()} → "
                f"{second_side.upper()} // {elapsed:.1f} s"
            )
        else:
            self.page2_status_var.set(
                f"PAGE 02 SERIAL FINISHED // "
                f"Check {second_side.upper()} status/telemetry"
            )

        self._finish_page2_dual_sequence()

    def _page2_program_chunks(self, side):
        """
        Split one arm's workflow into chunks that END at a Program step.

        This preserves setup steps that belong before that trajectory:
          Pole / Hand / Wait / Global / Program

        Example:
          Pole, Hand, Global, Program10, Program11, Pole, Program12

        becomes:
          chunk 1 = Pole + Hand + Global + Program10
          chunk 2 = Program11
          chunk 3 = Pole + Program12
        """
        builder = self._page2_builder(side)
        chunks = []
        pending = []

        for step in builder.steps:
            pending.append(json.loads(json.dumps(step)))

            if step.get("type") == "program":
                program = step
                chunks.append({
                    "side": side,
                    "steps": pending,
                    "program_id": int(program.get("id")),
                    "program_name": str(program.get("name") or ""),
                    "dual_rule": str(
                        program.get("dual_rule") or "TOGETHER"
                    ).upper(),
                })
                pending = []

        # Preserve any trailing non-program actions after the final Program.
        if pending:
            chunks.append({
                "side": side,
                "steps": pending,
                "program_id": None,
                "program_name": "FINAL AUX STEPS",
                "dual_rule": "TOGETHER",
            })

        return chunks

    def _page2_build_program_rule_plan(self):
        """
        Build the Page-02 dual-arm plan.

        v53 SAFE TIMELINE RULES

        TOGETHER sections may contain different Program IDs and different
        numbers of Program rows on LEFT and RIGHT.

        A positive WAIT located before a TOGETHER Program on BOTH arms is
        treated as a HARD CROSS-ARM CHECKPOINT:

          1. finish every preceding timeline Program on BOTH arms,
          2. run the checkpoint setup steps (for example Pole 150 + Wait 2 s)
             on BOTH arms,
          3. stop and require operator confirmation,
          4. only then start the Program after the WAIT on BOTH arms.

        This prevents one arm from entering the post-lid-cleaning Program
        while the other arm is still finishing the pre-lid section.

        RUN FIRST + WAIT OTHER remains an explicit serial handoff and still
        requires the same Program ID on both arms.
        """
        left_chunks = self._page2_program_chunks("left")
        right_chunks = self._page2_program_chunks("right")

        left_programs = [
            x for x in left_chunks if x["program_id"] is not None
        ]
        right_programs = [
            x for x in right_chunks if x["program_id"] is not None
        ]

        def validate_unique(items, side):
            seen = set()
            for item in items:
                pid = item["program_id"]
                if pid in seen:
                    raise ValueError(
                        f"{side.upper()} Program {pid} appears more than once. "
                        "SAFE TIMELINE requires unique Program IDs per arm."
                    )
                seen.add(pid)

        validate_unique(left_programs, "left")
        validate_unique(right_programs, "right")

        def rule(item):
            return str(item.get("dual_rule") or "TOGETHER").upper()

        def positive_wait_index(item):
            found = None
            for idx, step in enumerate(item.get("steps") or []):
                if str(step.get("type") or "").lower() != "wait":
                    continue
                try:
                    seconds = float(step.get("seconds", step.get("wait_s", 0)) or 0)
                except Exception:
                    seconds = 0.0
                if seconds > 0:
                    found = idx
            return found

        def combine(side, chunks, label="TOGETHER"):
            steps = []
            ids = []
            names = []

            for chunk in chunks:
                steps.extend(json.loads(json.dumps(chunk["steps"])))
                if chunk.get("program_id") is not None:
                    ids.append(int(chunk["program_id"]))
                if chunk.get("program_name"):
                    names.append(str(chunk["program_name"]))

            return {
                "side": side,
                "steps": steps,
                "program_id": ids[-1] if ids else None,
                "program_ids": ids,
                "program_name": " + ".join(names) or label,
                "dual_rule": "TOGETHER",
            }

        def split_wait_chunk(item):
            wi = positive_wait_index(item)
            if wi is None:
                raise ValueError("internal SAFE TIMELINE error: checkpoint WAIT not found")

            before = {
                "side": item["side"],
                "steps": json.loads(json.dumps(item["steps"][: wi + 1])),
                "program_id": None,
                "program_ids": [],
                "program_name": "MANUAL LID PREP",
                "dual_rule": "TOGETHER",
            }

            after_steps = json.loads(json.dumps(item["steps"][wi + 1 :]))
            if not after_steps:
                raise ValueError(
                    f"{item['side'].upper()} checkpoint WAIT has no Program after it."
                )

            programs_after = [
                s for s in after_steps if str(s.get("type") or "").lower() == "program"
            ]
            if len(programs_after) != 1:
                raise ValueError(
                    f"{item['side'].upper()} checkpoint must lead to exactly one "
                    "Program in that chunk."
                )

            program = programs_after[0]
            after = {
                "side": item["side"],
                "steps": after_steps,
                "program_id": int(program["id"]),
                "program_ids": [int(program["id"])],
                "program_name": str(program.get("name") or ""),
                "dual_rule": str(program.get("dual_rule") or "TOGETHER").upper(),
            }
            return before, after

        plan = []
        li = 0
        ri = 0
        timeline_number = 0
        checkpoint_number = 0

        while li < len(left_programs) or ri < len(right_programs):
            lcur = left_programs[li] if li < len(left_programs) else None
            rcur = right_programs[ri] if ri < len(right_programs) else None

            # ----------------------------------------------------------
            # TOGETHER region
            # ----------------------------------------------------------
            if (
                lcur is not None
                and rcur is not None
                and rule(lcur) == "TOGETHER"
                and rule(rcur) == "TOGETHER"
            ):
                # Locate the next WAIT checkpoint before the next serial rule.
                lbar = None
                j = li
                while j < len(left_programs) and rule(left_programs[j]) == "TOGETHER":
                    if positive_wait_index(left_programs[j]) is not None:
                        lbar = j
                        break
                    j += 1

                rbar = None
                j = ri
                while j < len(right_programs) and rule(right_programs[j]) == "TOGETHER":
                    if positive_wait_index(right_programs[j]) is not None:
                        rbar = j
                        break
                    j += 1

                if (lbar is None) != (rbar is None):
                    raise ValueError(
                        "SAFE TIMELINE checkpoint mismatch.\n\n"
                        "A positive WAIT exists before the next serial section "
                        "on only one arm. Add the manual checkpoint WAIT to BOTH "
                        "arms at the same process boundary."
                    )

                if lbar is not None and rbar is not None:
                    # Everything BEFORE the checkpoint must complete on BOTH arms.
                    left_pre = left_programs[li:lbar]
                    right_pre = right_programs[ri:rbar]

                    if bool(left_pre) != bool(right_pre):
                        raise ValueError(
                            "SAFE TIMELINE checkpoint alignment is ambiguous.\n\n"
                            f"LEFT Programs before checkpoint: "
                            f"{[x['program_id'] for x in left_pre] or 'none'}\n"
                            f"RIGHT Programs before checkpoint: "
                            f"{[x['program_id'] for x in right_pre] or 'none'}"
                        )

                    if left_pre and right_pre:
                        timeline_number += 1
                        left_item = combine("left", left_pre)
                        right_item = combine("right", right_pre)
                        plan.append({
                            "kind": "parallel",
                            "program_id": f"TIMELINE-{timeline_number}",
                            "timeline_number": timeline_number,
                            "left_program_ids": left_item["program_ids"],
                            "right_program_ids": right_item["program_ids"],
                            "left": left_item,
                            "right": right_item,
                        })

                    # Split the checkpoint chunk:
                    # setup/pole/wait -> MANUAL CONFIRM -> Program.
                    left_prep, left_after = split_wait_chunk(left_programs[lbar])
                    right_prep, right_after = split_wait_chunk(right_programs[rbar])

                    if rule(left_after) != "TOGETHER" or rule(right_after) != "TOGETHER":
                        raise ValueError(
                            "Program after the manual checkpoint must be TOGETHER "
                            "on both arms."
                        )

                    checkpoint_number += 1

                    plan.append({
                        "kind": "parallel",
                        "program_id": f"LID-PREP-{checkpoint_number}",
                        "timeline_number": None,
                        "left_program_ids": [],
                        "right_program_ids": [],
                        "left": left_prep,
                        "right": right_prep,
                    })

                    plan.append({
                        "kind": "checkpoint",
                        "program_id": f"LID-CHECK-{checkpoint_number}",
                        "checkpoint_number": checkpoint_number,
                        "title": "MANUAL LID COVER CHECKPOINT",
                        "message": (
                            "Both pre-lid cleaning sections are complete and both "
                            "checkpoint setup sequences have finished.\n\n"
                            "Open the toilet lid cover manually.\n\n"
                            "Before continuing, confirm:\n"
                            "• Lid cover is fully open\n"
                            "• Both hands are completely outside the robot workspace\n"
                            "• Both robot paths are clear\n\n"
                            "Press OK only when it is safe to start the next robot Programs."
                        ),
                    })

                    timeline_number += 1
                    plan.append({
                        "kind": "parallel",
                        "program_id": f"TIMELINE-{timeline_number}",
                        "timeline_number": timeline_number,
                        "left_program_ids": left_after["program_ids"],
                        "right_program_ids": right_after["program_ids"],
                        "left": left_after,
                        "right": right_after,
                    })

                    li = lbar + 1
                    ri = rbar + 1
                    continue

                # No checkpoint before the next serial section:
                # take all consecutive TOGETHER chunks on each arm.
                left_block = []
                while li < len(left_programs) and rule(left_programs[li]) == "TOGETHER":
                    left_block.append(left_programs[li])
                    li += 1

                right_block = []
                while ri < len(right_programs) and rule(right_programs[ri]) == "TOGETHER":
                    right_block.append(right_programs[ri])
                    ri += 1

                if not left_block or not right_block:
                    raise ValueError(
                        "SAFE TIMELINE found a TOGETHER section on only one arm."
                    )

                timeline_number += 1
                left_item = combine("left", left_block)
                right_item = combine("right", right_block)
                plan.append({
                    "kind": "parallel",
                    "program_id": f"TIMELINE-{timeline_number}",
                    "timeline_number": timeline_number,
                    "left_program_ids": left_item["program_ids"],
                    "right_program_ids": right_item["program_ids"],
                    "left": left_item,
                    "right": right_item,
                })
                continue

            # ----------------------------------------------------------
            # Serial RUN FIRST / WAIT OTHER
            # ----------------------------------------------------------
            if lcur is None or rcur is None:
                raise ValueError(
                    "SAFE TIMELINE reached an unmatched Program section.\n\n"
                    f"Next LEFT Program: "
                    f"{None if lcur is None else lcur['program_id']}\n"
                    f"Next RIGHT Program: "
                    f"{None if rcur is None else rcur['program_id']}"
                )

            lpid = int(lcur["program_id"])
            rpid = int(rcur["program_id"])
            lr = rule(lcur)
            rr = rule(rcur)

            if lpid != rpid:
                raise ValueError(
                    "RUN FIRST / WAIT OTHER serial handoffs must use the same "
                    "Program ID on both arms.\n\n"
                    f"LEFT  = Program {lpid} ({lr})\n"
                    f"RIGHT = Program {rpid} ({rr})"
                )

            if lr == "RUN FIRST" and rr == "WAIT OTHER":
                plan.append({
                    "kind": "serial",
                    "program_id": lpid,
                    "first": lcur,
                    "second": rcur,
                })
            elif lr == "WAIT OTHER" and rr == "RUN FIRST":
                plan.append({
                    "kind": "serial",
                    "program_id": lpid,
                    "first": rcur,
                    "second": lcur,
                })
            else:
                raise ValueError(
                    f"Program {lpid} has an invalid Dual Rule pair:\n\n"
                    f"LEFT  = {lr}\n"
                    f"RIGHT = {rr}\n\n"
                    "Use TOGETHER on both arms, or RUN FIRST + WAIT OTHER."
                )

            li += 1
            ri += 1

        # Preserve trailing non-program steps after the final Program.
        left_tail = next(
            (x for x in left_chunks if x["program_id"] is None),
            None,
        )
        right_tail = next(
            (x for x in right_chunks if x["program_id"] is None),
            None,
        )

        if left_tail or right_tail:
            plan.append({
                "kind": "tail",
                "program_id": None,
                "left": left_tail,
                "right": right_tail,
            })

        return plan

    def _page2_set_live(self, side, text):
        value = f"{side.upper()} // {text}"
        if side == "left":
            self.page2_left_live_var.set(value)
        else:
            self.page2_right_live_var.set(value)

    def _page2_program_live(self, side, pid, state):
        self._page2_set_live(side, f"Program {pid} // {state}")

    def _start_page2_program_rules(self):
        try:
            self.page2_rule_plan = self._page2_build_program_rule_plan()
        except Exception as exc:
            messagebox.showerror(
                "Program Rules",
                str(exc),
                parent=self,
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_rule_index = 0
        self.page2_rule_stage = None
        self.page2_rule_current = None

        self.page2_status_var.set(
            f"PAGE 02 PROGRAM RULES // "
            f"{len(self.page2_rule_plan)} coordinated stage(s)"
        )
        self._page2_set_live("left", "READY")
        self._page2_set_live("right", "READY")
        self.page2_stop_both_btn.configure(state="normal")

        self._start_next_page2_rule_stage()

    def _page2_rule_chunk_command(self, item):
        if item is None:
            return None

        side = item["side"]
        arm_panel = self._page2_arm_panel(side)
        env = arm_panel._validate_environment()

        if not env:
            return None

        rm_upload, sdk = env

        folder = Path(tempfile.gettempdir()) / "realman_dual_sequence_ui"
        folder.mkdir(parents=True, exist_ok=True)

        pid = item["program_id"]
        suffix = str(pid) if pid is not None else "tail"
        path = folder / (
            f"{side}_rule_{self.page2_rule_index:02d}_{suffix}.json"
        )

        doc = {
            "format": "realman_workflow_sequence_v1",
            "side": side,
            "steps": item["steps"],
        }

        path.write_text(
            json.dumps(doc, indent=2),
            encoding="utf-8",
        )

        return arm_panel._base_command(rm_upload, sdk) + [
            "--workflow-sequence-file",
            str(path),
            "--workflow-overall-speed",
            str(self._page2_overall_speed()),
            "--side",
            side,
            "--yes",
        ]

    def _start_next_page2_rule_stage(self):
        if not self.page2_dual_active:
            return

        if self.page2_rule_index >= len(self.page2_rule_plan):
            elapsed = (
                __import__("time").monotonic() - self.page2_dual_started_at
                if self.page2_dual_started_at is not None
                else 0.0
            )
            self.page2_runtime_var.set(f"{elapsed:.1f} s")
            self.page2_status_var.set(
                f"PAGE 02 PROGRAM RULES COMPLETE // {elapsed:.1f} s"
            )
            self._page2_set_live("left", "SEQUENCE COMPLETE ✓")
            self._page2_set_live("right", "SEQUENCE COMPLETE ✓")
            self._finish_page2_dual_sequence()
            return

        # Do not launch any new stage until both controllers are idle.
        if (
            self.left_panel.has_active_command()
            or self.right_panel.has_active_command()
        ):
            self.page2_dual_after_id = self.after(
                100,
                self._start_next_page2_rule_stage,
            )
            return

        stage = self.page2_rule_plan[self.page2_rule_index]
        self.page2_rule_current = stage
        kind = stage["kind"]

        if kind == "parallel":
            self._start_page2_rule_parallel(stage)
        elif kind == "serial":
            self._start_page2_rule_serial_first(stage)
        elif kind == "checkpoint":
            self._start_page2_manual_checkpoint(stage)
        else:
            self._start_page2_rule_tail(stage)

    def _start_page2_manual_checkpoint(self, stage):
        """
        Hard operator gate between the pre-lid and post-lid robot sections.

        No robot command is active when this dialog is shown. The sequence
        cannot continue until the operator explicitly confirms.
        """
        self.page2_rule_stage = "CHECKPOINT"

        self.page2_status_var.set(
            "SAFE TIMELINE // MANUAL LID COVER CHECKPOINT // ROBOTS PAUSED"
        )
        self._page2_set_live("left", "PAUSED // MANUAL LID CHECK")
        self._page2_set_live("right", "PAUSED // MANUAL LID CHECK")
        self.left_sequence_panel.set_running(False)
        self.right_sequence_panel.set_running(False)

        ok = messagebox.askokcancel(
            stage.get("title") or "Manual Checkpoint",
            stage.get("message") or (
                "Complete the manual operation, clear the robot workspace, "
                "then press OK to continue."
            ),
            icon="warning",
            parent=self,
        )

        if not ok:
            self.page2_status_var.set(
                "SAFE TIMELINE CANCELLED // MANUAL CHECKPOINT NOT CONFIRMED"
            )
            self._page2_set_live("left", "IDLE // CHECKPOINT CANCELLED")
            self._page2_set_live("right", "IDLE // CHECKPOINT CANCELLED")
            self._finish_page2_dual_sequence()
            return

        self.page2_status_var.set(
            "SAFE TIMELINE // MANUAL LID CONFIRMED // RESUMING"
        )
        self._page2_set_live("left", "READY // LID CONFIRMED")
        self._page2_set_live("right", "READY // LID CONFIRMED")

        self.page2_rule_index += 1
        self.page2_rule_current = None
        self.page2_dual_after_id = self.after(
            100,
            self._start_next_page2_rule_stage,
        )

    def _start_page2_rule_parallel(self, stage):
        left = stage["left"]
        right = stage["right"]
        pid = stage["program_id"]

        try:
            left_cmd = self._page2_rule_chunk_command(left)
            right_cmd = self._page2_rule_chunk_command(right)
        except Exception as exc:
            self.page2_status_var.set(
                f"PROGRAM RULES FAULT // Program {pid} // {exc}"
            )
            self._finish_page2_dual_sequence()
            return

        if not left_cmd or not right_cmd:
            self._finish_page2_dual_sequence()
            return

        barrier = threading.Barrier(2)

        self.page2_rule_stage = "PARALLEL"
        self.page2_status_var.set(
            f"PROGRAM RULES // Program {pid} // LEFT + RIGHT TOGETHER"
        )
        self._page2_program_live("left", pid, "RUNNING")
        self._page2_program_live("right", pid, "RUNNING")
        self.left_sequence_panel.set_running(True)
        self.right_sequence_panel.set_running(True)

        try:
            self.left_panel._start_process_synced(
                left_cmd,
                f"PROGRAM RULES // LEFT Program {pid}",
                barrier,
            )
            self.right_panel._start_process_synced(
                right_cmd,
                f"PROGRAM RULES // RIGHT Program {pid}",
                barrier,
            )
        except Exception as exc:
            self.page2_status_var.set(
                f"PROGRAM RULES FAULT // Program {pid} // {exc}"
            )
            self.left_panel._slow_stop()
            self.right_panel._slow_stop()
            self._finish_page2_dual_sequence()
            return

        self.page2_dual_after_id = self.after(
            120,
            self._monitor_page2_rule_parallel,
        )

    def _monitor_page2_rule_parallel(self):
        if not self.page2_dual_active:
            return

        self._page2_update_rule_runtime()

        left_active = self.left_panel.has_active_command()
        right_active = self.right_panel.has_active_command()
        pid = self.page2_rule_current.get("program_id")

        if left_active:
            self._page2_program_live("left", pid, "RUNNING")
        elif self.left_panel.last_command_rc == 0:
            self._page2_program_live("left", pid, "COMPLETE ✓")
        elif self.left_panel.last_command_rc is not None:
            self._page2_program_live(
                "left", pid, f"FAILED rc={self.left_panel.last_command_rc}"
            )

        if right_active:
            self._page2_program_live("right", pid, "RUNNING")
        elif self.right_panel.last_command_rc == 0:
            self._page2_program_live("right", pid, "COMPLETE ✓")
        elif self.right_panel.last_command_rc is not None:
            self._page2_program_live(
                "right", pid, f"FAILED rc={self.right_panel.last_command_rc}"
            )

        if left_active or right_active:
            self.page2_dual_after_id = self.after(
                120,
                self._monitor_page2_rule_parallel,
            )
            return

        self.left_sequence_panel.set_running(False)
        self.right_sequence_panel.set_running(False)

        left_ok = self._page2_command_succeeded(self.left_panel)
        right_ok = self._page2_command_succeeded(self.right_panel)

        if not left_ok or not right_ok:
            if not left_ok:
                self._page2_program_live(
                    "left", pid, f"FAILED rc={self.left_panel.last_command_rc}"
                )
            if not right_ok:
                self._page2_program_live(
                    "right", pid, f"FAILED rc={self.right_panel.last_command_rc}"
                )
            self.page2_status_var.set(
                "PROGRAM RULES STOPPED // parallel Program stage failed // "
                f"LEFT rc={self.left_panel.last_command_rc} // "
                f"RIGHT rc={self.right_panel.last_command_rc}"
            )
            self._finish_page2_dual_sequence()
            return

        pid = self.page2_rule_current.get("program_id")
        self._page2_program_live("left", pid, "COMPLETE ✓")
        self._page2_program_live("right", pid, "COMPLETE ✓")
        next_index = self.page2_rule_index + 1
        if next_index < len(self.page2_rule_plan):
            next_stage = self.page2_rule_plan[next_index]
            next_pid = next_stage.get("program_id")
            if next_pid is not None:
                self.page2_status_var.set(
                    f"PROGRAM RULES // Program {pid} COMPLETE // "
                    f"NEXT Program {next_pid}"
                )

        self.page2_rule_index += 1
        self.page2_rule_current = None
        self.page2_dual_after_id = self.after(
            20,
            self._start_next_page2_rule_stage,
        )

    def _start_page2_rule_serial_first(self, stage):
        first = stage["first"]
        second = stage["second"]
        pid = stage["program_id"]

        side = first["side"]
        panel = self._page2_arm_panel(side)

        try:
            cmd = self._page2_rule_chunk_command(first)
        except Exception as exc:
            self.page2_status_var.set(
                f"PROGRAM RULES FAULT // Program {pid} // {exc}"
            )
            self._finish_page2_dual_sequence()
            return

        if not cmd:
            self._finish_page2_dual_sequence()
            return

        self.page2_rule_stage = "SERIAL_FIRST"
        self.page2_status_var.set(
            f"PROGRAM RULES // Program {pid} // "
            f"{side.upper()} RUN FIRST // "
            f"{second['side'].upper()} WAITING"
        )
        self._page2_program_live(first["side"], pid, "RUNNING // RUN FIRST")
        self._page2_program_live(
            second["side"], pid, f"WAITING FOR {first['side'].upper()}"
        )

        self.left_sequence_panel.set_running(side == "left")
        self.right_sequence_panel.set_running(side == "right")

        try:
            panel._start_process(
                cmd,
                f"PROGRAM RULES // {side.upper()} Program {pid} FIRST",
                motion=True,
            )
        except Exception as exc:
            self.page2_status_var.set(
                f"PROGRAM RULES FAULT // Program {pid} // {exc}"
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_dual_after_id = self.after(
            120,
            self._monitor_page2_rule_serial_first,
        )

    def _monitor_page2_rule_serial_first(self):
        if not self.page2_dual_active:
            return

        self._page2_update_rule_runtime()

        stage = self.page2_rule_current
        first = stage["first"]
        second = stage["second"]
        pid = stage["program_id"]

        first_panel = self._page2_arm_panel(first["side"])

        if first_panel.has_active_command():
            self._page2_program_live(
                first["side"], pid, "RUNNING // RUN FIRST"
            )
            self._page2_program_live(
                second["side"], pid, f"WAITING FOR {first['side'].upper()}"
            )
            self.page2_dual_after_id = self.after(
                120,
                self._monitor_page2_rule_serial_first,
            )
            return

        self.left_sequence_panel.set_running(False)
        self.right_sequence_panel.set_running(False)

        if not self._page2_command_succeeded(first_panel):
            self._page2_program_live(
                first["side"], pid, f"FAILED rc={first_panel.last_command_rc}"
            )
            self._page2_program_live(second["side"], pid, "NOT STARTED")
            self.page2_status_var.set(
                f"PROGRAM RULES STOPPED // "
                f"{first['side'].upper()} Program {pid} failed "
                f"(rc={first_panel.last_command_rc}) // "
                f"{second['side'].upper()} NOT STARTED"
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_status_var.set(
            f"PROGRAM RULES // {first['side'].upper()} Program {pid} COMPLETE // "
            f"{second['side'].upper()} Program {pid} WAITING"
        )
        self._page2_program_live(first["side"], pid, "COMPLETE ✓")
        if self.page2_alt_confirm_each_var.get():
            self._page2_program_live(
                second["side"], pid, "WAITING // CONFIRMATION REQUIRED"
            )
        else:
            self._page2_program_live(
                second["side"], pid, "WAITING // AUTO HANDOFF"
            )

        if self.page2_alt_confirm_each_var.get():
            proceed = messagebox.askyesno(
                "Program Handoff",
                f"{first['side'].upper()} Program {pid} completed.\\n\\n"
                f"{second['side'].upper()} Program {pid} is waiting.\\n\\n"
                f"Start {second['side'].upper()} Program {pid} now?",
                icon="warning",
                parent=self,
            )

            if not proceed:
                self._page2_program_live(
                    second["side"], pid, "HANDOFF CANCELLED"
                )
                self.page2_status_var.set(
                    f"PROGRAM RULES PAUSED // Program {pid} handoff cancelled"
                )
                self._finish_page2_dual_sequence()
                return

        self._start_page2_rule_serial_second(stage)

    def _start_page2_rule_serial_second(self, stage):
        second = stage["second"]
        pid = stage["program_id"]
        side = second["side"]
        panel = self._page2_arm_panel(side)

        try:
            cmd = self._page2_rule_chunk_command(second)
        except Exception as exc:
            self.page2_status_var.set(
                f"PROGRAM RULES FAULT // Program {pid} // {exc}"
            )
            self._finish_page2_dual_sequence()
            return

        if not cmd:
            self._finish_page2_dual_sequence()
            return

        self.page2_rule_stage = "SERIAL_SECOND"
        self.page2_status_var.set(
            f"PROGRAM RULES // Program {pid} // "
            f"{side.upper()} STARTING AFTER OTHER ARM"
        )
        first_side = stage["first"]["side"]
        self._page2_program_live(first_side, pid, "COMPLETE ✓")
        self._page2_program_live(side, pid, "RUNNING // SECOND")

        self.left_sequence_panel.set_running(side == "left")
        self.right_sequence_panel.set_running(side == "right")

        try:
            panel._start_process(
                cmd,
                f"PROGRAM RULES // {side.upper()} Program {pid} SECOND",
                motion=True,
            )
        except Exception as exc:
            self.page2_status_var.set(
                f"PROGRAM RULES FAULT // Program {pid} // {exc}"
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_dual_after_id = self.after(
            120,
            self._monitor_page2_rule_serial_second,
        )

    def _monitor_page2_rule_serial_second(self):
        if not self.page2_dual_active:
            return

        self._page2_update_rule_runtime()

        stage = self.page2_rule_current
        second = stage["second"]
        pid = stage["program_id"]
        panel = self._page2_arm_panel(second["side"])

        if panel.has_active_command():
            self._page2_program_live(
                second["side"], pid, "RUNNING // SECOND"
            )
            self.page2_dual_after_id = self.after(
                120,
                self._monitor_page2_rule_serial_second,
            )
            return

        self.left_sequence_panel.set_running(False)
        self.right_sequence_panel.set_running(False)

        if not self._page2_command_succeeded(panel):
            self._page2_program_live(
                second["side"], pid, f"FAILED rc={panel.last_command_rc}"
            )
            self.page2_status_var.set(
                f"PROGRAM RULES STOPPED // "
                f"{second['side'].upper()} Program {pid} failed "
                f"(rc={panel.last_command_rc})"
            )
            self._finish_page2_dual_sequence()
            return

        self._page2_program_live(second["side"], pid, "COMPLETE ✓")

        # The two-part serial Program stage is now fully complete.
        # Advance immediately to the next Program-rule stage. This is the
        # important handoff for cases such as:
        #   LEFT 8 -> RIGHT 8 -> LEFT 9 -> RIGHT 9 -> BOTH 10
        next_index = self.page2_rule_index + 1
        if next_index < len(self.page2_rule_plan):
            next_stage = self.page2_rule_plan[next_index]
            next_pid = next_stage.get("program_id")
            next_kind = next_stage.get("kind")

            if next_pid is not None:
                if next_kind == "parallel":
                    next_text = f"LEFT + RIGHT Program {next_pid} TOGETHER"
                else:
                    next_text = f"Program {next_pid}"

                self.page2_status_var.set(
                    f"PROGRAM RULES // Program {pid} SERIAL COMPLETE // "
                    f"NEXT: {next_text}"
                )

        self.page2_rule_index += 1
        self.page2_rule_current = None
        self.page2_dual_after_id = self.after(
            40,
            self._start_next_page2_rule_stage,
        )

    def _start_page2_rule_tail(self, stage):
        left = stage.get("left")
        right = stage.get("right")

        # If both arms have trailing non-program steps, run them together.
        if left and right:
            try:
                left_cmd = self._page2_rule_chunk_command(left)
                right_cmd = self._page2_rule_chunk_command(right)
            except Exception as exc:
                self.page2_status_var.set(
                    f"PROGRAM RULES FINAL STEPS FAULT // {exc}"
                )
                self._finish_page2_dual_sequence()
                return

            barrier = threading.Barrier(2)
            self.page2_rule_stage = "TAIL_BOTH"
            self.page2_status_var.set(
                "PROGRAM RULES // FINAL AUX STEPS // LEFT + RIGHT"
            )
            self._page2_set_live("left", "FINAL AUX // RUNNING")
            self._page2_set_live("right", "FINAL AUX // RUNNING")
            self.left_sequence_panel.set_running(True)
            self.right_sequence_panel.set_running(True)

            try:
                self.left_panel._start_process_synced(
                    left_cmd,
                    "PROGRAM RULES // LEFT FINAL AUX",
                    barrier,
                )
                self.right_panel._start_process_synced(
                    right_cmd,
                    "PROGRAM RULES // RIGHT FINAL AUX",
                    barrier,
                )
            except Exception as exc:
                self.page2_status_var.set(
                    f"PROGRAM RULES FINAL STEPS FAULT // {exc}"
                )
                self.left_panel._slow_stop()
                self.right_panel._slow_stop()
                self._finish_page2_dual_sequence()
                return

            self.page2_dual_after_id = self.after(
                120,
                self._monitor_page2_rule_tail,
            )
            return

        item = left or right
        if not item:
            self.page2_rule_index += 1
            self._start_next_page2_rule_stage()
            return

        side = item["side"]
        panel = self._page2_arm_panel(side)

        try:
            cmd = self._page2_rule_chunk_command(item)
        except Exception as exc:
            self.page2_status_var.set(
                f"PROGRAM RULES FINAL STEPS FAULT // {exc}"
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_rule_stage = "TAIL_ONE"
        self.page2_status_var.set(
            f"PROGRAM RULES // {side.upper()} FINAL AUX STEPS"
        )
        self._page2_set_live(side, "FINAL AUX // RUNNING")
        other = "right" if side == "left" else "left"
        self._page2_set_live(other, "IDLE")
        self.left_sequence_panel.set_running(side == "left")
        self.right_sequence_panel.set_running(side == "right")

        try:
            panel._start_process(
                cmd,
                f"PROGRAM RULES // {side.upper()} FINAL AUX",
                motion=True,
            )
        except Exception as exc:
            self.page2_status_var.set(
                f"PROGRAM RULES FINAL STEPS FAULT // {exc}"
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_dual_after_id = self.after(
            120,
            self._monitor_page2_rule_tail,
        )

    def _monitor_page2_rule_tail(self):
        if not self.page2_dual_active:
            return

        self._page2_update_rule_runtime()

        if (
            self.left_panel.has_active_command()
            or self.right_panel.has_active_command()
        ):
            self.page2_dual_after_id = self.after(
                120,
                self._monitor_page2_rule_tail,
            )
            return

        self.left_sequence_panel.set_running(False)
        self.right_sequence_panel.set_running(False)

        # Only check whichever arm(s) actually had a tail.
        stage = self.page2_rule_current
        if stage.get("left"):
            if not self._page2_command_succeeded(self.left_panel):
                self.page2_status_var.set(
                    "PROGRAM RULES STOPPED // LEFT final auxiliary steps failed "
                    f"(rc={self.left_panel.last_command_rc})"
                )
                self._finish_page2_dual_sequence()
                return

        if stage.get("right"):
            if not self._page2_command_succeeded(self.right_panel):
                self.page2_status_var.set(
                    "PROGRAM RULES STOPPED // RIGHT final auxiliary steps failed "
                    f"(rc={self.right_panel.last_command_rc})"
                )
                self._finish_page2_dual_sequence()
                return

        self.page2_rule_index += 1
        self.page2_rule_current = None
        self.page2_dual_after_id = self.after(
            20,
            self._start_next_page2_rule_stage,
        )

    @staticmethod
    def _page2_command_succeeded(panel):
        """Use subprocess return code, not asynchronously-updated status text."""
        return getattr(panel, "last_command_rc", None) == 0

    @staticmethod
    def _page2_command_failed(panel):
        rc = getattr(panel, "last_command_rc", None)
        return rc is not None and rc != 0

    def _page2_update_rule_runtime(self):
        if self.page2_dual_started_at is None:
            return
        elapsed = (
            __import__("time").monotonic()
            - self.page2_dual_started_at
        )
        self.page2_runtime_var.set(f"{elapsed:.1f} s")

    def _page2_build_alt_queue(self, first_side):
        left = self._page2_program_chunks("left")
        right = self._page2_program_chunks("right")

        # Require at least one actual Program per side.
        left_programs = [x for x in left if x["program_id"] is not None]
        right_programs = [x for x in right if x["program_id"] is not None]

        if not left_programs or not right_programs:
            raise ValueError(
                "ALT PROGRAMS mode needs at least one PROGRAM step "
                "in both LEFT and RIGHT sequences."
            )

        first = left if first_side == "left" else right
        second = right if first_side == "left" else left

        queue = []
        count = max(len(first), len(second))

        for i in range(count):
            if i < len(first):
                queue.append(first[i])
            if i < len(second):
                queue.append(second[i])

        return queue

    def _page2_write_chunk(self, item):
        side = item["side"]
        folder = Path(tempfile.gettempdir()) / "realman_dual_sequence_ui"
        folder.mkdir(parents=True, exist_ok=True)

        pid = item["program_id"]
        suffix = str(pid) if pid is not None else "tail"
        path = folder / f"{side}_alt_{self.page2_alt_index:02d}_{suffix}.json"

        doc = {
            "format": "realman_workflow_sequence_v1",
            "side": side,
            "steps": item["steps"],
        }

        path.write_text(
            json.dumps(doc, indent=2),
            encoding="utf-8",
        )
        return path

    def _page2_alt_command(self, item):
        side = item["side"]
        arm_panel = self._page2_arm_panel(side)
        env = arm_panel._validate_environment()

        if not env:
            return None

        rm_upload, sdk = env
        sequence_path = self._page2_write_chunk(item)

        return arm_panel._base_command(rm_upload, sdk) + [
            "--workflow-sequence-file",
            str(sequence_path),
            "--workflow-overall-speed",
            str(self._page2_overall_speed()),
            "--side",
            side,
            "--yes",
        ]

    def _start_page2_alt_programs(self, first_side):
        try:
            self.page2_alt_queue = self._page2_build_alt_queue(first_side)
        except Exception as exc:
            messagebox.showerror(
                "Alternate Programs",
                str(exc),
                parent=self,
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_alt_index = 0
        self.page2_alt_current = None

        total_programs = sum(
            1
            for item in self.page2_alt_queue
            if item["program_id"] is not None
        )

        self.page2_status_var.set(
            f"PAGE 02 ALT PROGRAMS // "
            f"{len(self.page2_alt_queue)} chunk(s) // "
            f"{total_programs} trajectory program(s)"
        )

        self.page2_stop_both_btn.configure(state="normal")
        self._start_next_page2_alt_chunk()

    def _alt_item_text(self, item):
        side = item["side"].upper()
        pid = item["program_id"]

        if pid is None:
            return f"{side} final auxiliary steps"

        name = item.get("program_name") or ""
        return f"{side} Program {pid}" + (f" // {name}" if name else "")

    def _start_next_page2_alt_chunk(self):
        if not self.page2_dual_active:
            return

        if self.page2_alt_index >= len(self.page2_alt_queue):
            elapsed = (
                __import__("time").monotonic() - self.page2_dual_started_at
                if self.page2_dual_started_at is not None
                else 0.0
            )
            self.page2_runtime_var.set(f"{elapsed:.1f} s")
            self.page2_status_var.set(
                f"PAGE 02 ALT PROGRAMS COMPLETE // {elapsed:.1f} s"
            )
            self._finish_page2_dual_sequence()
            return

        item = self.page2_alt_queue[self.page2_alt_index]
        self.page2_alt_current = item
        side = item["side"]
        arm_panel = self._page2_arm_panel(side)

        # Hard safety rule: never launch while either robot has an active command.
        if (
            self.left_panel.has_active_command()
            or self.right_panel.has_active_command()
        ):
            self.page2_dual_after_id = self.after(
                100,
                self._start_next_page2_alt_chunk,
            )
            return

        # Optional confirmation BEFORE every chunk after the first one.
        if (
            self.page2_alt_confirm_each_var.get()
            and self.page2_alt_index > 0
        ):
            previous = self.page2_alt_queue[self.page2_alt_index - 1]
            proceed = messagebox.askyesno(
                "Continue Alternate Programs",
                f"COMPLETED:\\n{self._alt_item_text(previous)}\\n\\n"
                f"NEXT:\\n{self._alt_item_text(item)}\\n\\n"
                "Start the next arm/program now?",
                icon="warning",
                parent=self,
            )

            if not proceed:
                self.page2_status_var.set(
                    "PAGE 02 ALT PROGRAMS PAUSED // "
                    "Next program cancelled by operator"
                )
                self._finish_page2_dual_sequence()
                return

        try:
            cmd = self._page2_alt_command(item)
        except Exception as exc:
            self.page2_status_var.set(
                f"PAGE 02 ALT PROGRAMS FAULT // {exc}"
            )
            self._finish_page2_dual_sequence()
            return

        if not cmd:
            self._finish_page2_dual_sequence()
            return

        text = self._alt_item_text(item)
        self.page2_status_var.set(
            f"PAGE 02 ALT {self.page2_alt_index + 1}/"
            f"{len(self.page2_alt_queue)} // {text}"
        )

        self.left_sequence_panel.set_running(side == "left")
        self.right_sequence_panel.set_running(side == "right")

        try:
            arm_panel._start_process(
                cmd,
                f"PAGE 02 ALT PROGRAMS // {text}",
                motion=True,
            )
        except Exception as exc:
            self.page2_status_var.set(
                f"PAGE 02 ALT PROGRAMS FAULT // {exc}"
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_dual_after_id = self.after(
            120,
            self._monitor_page2_alt_chunk,
        )

    def _monitor_page2_alt_chunk(self):
        if not self.page2_dual_active:
            return

        now = __import__("time").monotonic()
        if self.page2_dual_started_at is not None:
            self.page2_runtime_var.set(
                f"{now - self.page2_dual_started_at:.1f} s"
            )

        item = self.page2_alt_current
        if item is None:
            self._finish_page2_dual_sequence()
            return

        side = item["side"]
        arm_panel = self._page2_arm_panel(side)

        if arm_panel.has_active_command():
            self.page2_dual_after_id = self.after(
                120,
                self._monitor_page2_alt_chunk,
            )
            return

        self.left_sequence_panel.set_running(False)
        self.right_sequence_panel.set_running(False)

        status = arm_panel.status_var.get()

        if not status.startswith("COMPLETE //"):
            self.page2_status_var.set(
                f"PAGE 02 ALT STOPPED // {self._alt_item_text(item)} "
                f"did not complete // other arm NOT started"
            )
            self._finish_page2_dual_sequence()
            return

        self.page2_alt_index += 1
        self.page2_alt_current = None

        # Small UI scheduling handoff only; no intentional robot wait.
        self.page2_dual_after_id = self.after(
            20,
            self._start_next_page2_alt_chunk,
        )

    def _run_both_page2_sequences(self):
        """
        Page 02 dual execution follows TIMELINE RULES. Consecutive TOGETHER
        rows form independent LEFT/RIGHT timeline blocks; serial RUN FIRST /
        WAIT OTHER rows remain paired handoffs.

        Valid pairs:
          TOGETHER  + TOGETHER   -> both run together
          RUN FIRST + WAIT OTHER -> RUN FIRST arm completes first
          WAIT OTHER + RUN FIRST -> opposite arm completes first
        """
        if self.page2_dual_active:
            return

        if (
            not self.left_sequence_panel.steps
            or not self.right_sequence_panel.steps
        ):
            messagebox.showinfo(
                "Dual Sequence",
                "Build or load BOTH LEFT and RIGHT sequences first.",
                parent=self,
            )
            return

        if (
            self.left_panel.has_active_command()
            or self.right_panel.has_active_command()
            or self.dual_run_active
            or self.dual_drag_active
            or self.dual_file_run_active
        ):
            messagebox.showwarning(
                "Controllers busy",
                "Both arms must be idle before RUN BOTH SEQUENCES.",
                parent=self,
            )
            return

        # Validate timeline blocks and serial Dual Rules BEFORE any robot moves.
        try:
            preview_plan = self._page2_build_program_rule_plan()
        except Exception as exc:
            messagebox.showerror(
                "Program Rules",
                f"Cannot start dual sequence:\n\n{exc}",
                parent=self,
            )
            return

        parallel_count = sum(
            1 for stage in preview_plan
            if stage.get("kind") == "parallel"
        )
        serial_count = sum(
            1 for stage in preview_plan
            if stage.get("kind") == "serial"
        )
        checkpoint_count = sum(
            1 for stage in preview_plan
            if stage.get("kind") == "checkpoint"
        )

        ok = messagebox.askyesno(
            "Run Dual Sequences",
            "DUAL SAFE-TIMELINE EXECUTION\n\n"
            f"Coordinated timeline/serial stages: {parallel_count + serial_count}\n"
            f"TOGETHER timeline blocks: {parallel_count}\n"
            f"One-arm-at-a-time stages: {serial_count}\n"
            f"Manual safety checkpoints: {checkpoint_count}\n"
            f"Overall controller speed: {self._page2_overall_speed()}%\n\n"
            "Consecutive TOGETHER rows run as synchronized LEFT/RIGHT timeline blocks. "
            "Program IDs may differ inside those blocks. A positive WAIT on both "
            "arms creates a hard cross-arm checkpoint and requires operator "
            "confirmation before the next robot Programs start.\n\n"
            "Confirm both work areas are clear and both hardware E-stops "
            "are accessible.\n\n"
            "START DUAL SEQUENCE?",
            icon="warning",
            parent=self,
        )
        if not ok:
            return

        self.page2_dual_active = True
        self.page2_dual_started_at = __import__("time").monotonic()
        self.page2_runtime_var.set("0.0 s")
        self._page2_set_live("left", "PREPARING")
        self._page2_set_live("right", "PREPARING")

        self.page2_run_both_btn.configure(state="disabled")
        self.page2_alt_confirm_check.configure(state="disabled")
        self.page2_overall_speed_entry.configure(state="disabled")

        # Use the already validated plan rather than rebuilding after approval.
        self.page2_rule_plan = preview_plan
        self.page2_rule_index = 0
        self.page2_rule_stage = None
        self.page2_rule_current = None

        self.page2_status_var.set(
            f"PAGE 02 PROGRAM RULES // "
            f"{len(self.page2_rule_plan)} coordinated stage(s)"
        )
        self.page2_stop_both_btn.configure(state="normal")

        self._start_next_page2_rule_stage()

    def _monitor_page2_dual_sequence(self):
        if not self.page2_dual_active:
            return

        now = __import__("time").monotonic()
        if self.page2_dual_started_at is not None:
            self.page2_runtime_var.set(
                f"{now - self.page2_dual_started_at:.1f} s"
            )

        left_active = self.left_panel.has_active_command()
        right_active = self.right_panel.has_active_command()

        if left_active or right_active:
            bits = [
                "LEFT ACTIVE" if left_active else "LEFT COMPLETE",
                "RIGHT ACTIVE" if right_active else "RIGHT COMPLETE",
            ]
            self.page2_status_var.set(
                "PAGE 02 // " + " // ".join(bits)
            )
            self.page2_dual_after_id = self.after(
                150,
                self._monitor_page2_dual_sequence,
            )
            return

        elapsed = (
            now - self.page2_dual_started_at
            if self.page2_dual_started_at is not None
            else 0.0
        )
        self.page2_runtime_var.set(f"{elapsed:.1f} s")

        left_status = self.left_panel.status_var.get()
        right_status = self.right_panel.status_var.get()

        if (
            left_status.startswith("COMPLETE //")
            and right_status.startswith("COMPLETE //")
        ):
            self.page2_status_var.set(
                f"PAGE 02 COMPLETE // Both sequences finished // {elapsed:.1f} s"
            )
        else:
            self.page2_status_var.set(
                "PAGE 02 FINISHED // Check LEFT and RIGHT status/telemetry"
            )

        self._finish_page2_dual_sequence()

    def _finish_page2_dual_sequence(self):
        self.page2_dual_active = False
        self.page2_dual_started_at = None
        self.page2_dual_after_id = None

        self.page2_serial_first_side = None
        self.page2_serial_second_side = None
        self.page2_serial_second_cmd = None

        self.page2_run_both_btn.configure(state="normal")
        self.page2_stop_both_btn.configure(state="disabled")
        self.page2_alt_confirm_check.configure(state="normal")
        self.page2_overall_speed_entry.configure(state="normal")

        self.page2_alt_queue = []
        self.page2_alt_index = 0
        self.page2_alt_current = None

        self.page2_rule_plan = []
        self.page2_rule_index = 0
        self.page2_rule_stage = None
        self.page2_rule_current = None

        self.left_sequence_panel.set_running(False)
        self.right_sequence_panel.set_running(False)

    def _stop_both_page2_sequences(self):
        # Immediate stop: no confirmation.
        # In LEFT → RIGHT / RIGHT → LEFT mode this also cancels the waiting arm.
        self.page2_status_var.set(
            "PAGE 02 STOP REQUESTED // LEFT + RIGHT"
        )
        self._page2_set_live("left", "STOP REQUESTED")
        self._page2_set_live("right", "STOP REQUESTED")

        self.page2_serial_second_cmd = None
        self.page2_alt_queue = []
        self.page2_rule_plan = []

        self.left_panel._slow_stop()
        self.right_panel._slow_stop()
        self.page2_stop_both_btn.configure(state="disabled")


    def _refresh_dual_combo_values(self):
        if not self.left_panel or not self.right_panel:
            return

        self.dual_left_combo.configure(
            values=self.left_panel._program_combo_values()
        )
        self.dual_right_combo.configure(
            values=self.right_panel._program_combo_values()
        )

    def _sync_both_program_lists(self):
        if (
            self.left_panel.has_active_command()
            or self.right_panel.has_active_command()
            or self.dual_run_active
            or self.dual_drag_active
        ):
            messagebox.showwarning(
                "Controllers busy",
                "Wait for both arms to become idle before refreshing both lists.",
                parent=self,
            )
            return

        self.dual_status_var.set(
            "SYNCING // Reading program lists from both controllers"
        )

        # Each panel uses its own process/thread and IP selected by --side.
        self.left_panel._list_slots()
        self.right_panel._list_slots()

        self.after(200, self._wait_for_dual_list_sync)

    def _wait_for_dual_list_sync(self):
        if (
            self.left_panel.has_active_command()
            or self.right_panel.has_active_command()
        ):
            self.after(200, self._wait_for_dual_list_sync)
            return

        # Allow queued _remember_program callbacks to finish.
        self.after(100, self._finish_dual_list_sync)

    def _finish_dual_list_sync(self):
        self._refresh_dual_combo_values()
        self.dual_status_var.set(
            f"SYNC COMPLETE // LEFT {len(self.left_panel.controller_programs)} programs"
            f" // RIGHT {len(self.right_panel.controller_programs)} programs"
        )

    @staticmethod
    def _dual_program_text(slot, name):
        return f"{slot}" + (f"  ({name})" if name else "")

    def _run_both_programs(self):
        if self.dual_file_run_active:
            messagebox.showwarning(
                "RUN BOTH .TXT active",
                "Wait until the current .txt run is complete.",
                parent=self,
            )
            return

        if self.dual_drag_active:
            messagebox.showwarning(
                "Dual Drag active",
                "Stop both Drag Teach sessions before starting Dual Run.",
                parent=self,
            )
            return

        if self.dual_run_active:
            messagebox.showwarning(
                "Dual Run active",
                "A synchronized Dual Run is already active.",
                parent=self,
            )
            return

        if self.left_panel.has_active_command() or self.right_panel.has_active_command():
            busy = []
            if self.left_panel.has_active_command():
                busy.append("LEFT")
            if self.right_panel.has_active_command():
                busy.append("RIGHT")

            messagebox.showwarning(
                "Arm busy",
                "Dual Run requires both arms to be idle.\n\nBusy: "
                + ", ".join(busy),
                parent=self,
            )
            return

        left_env = self.left_panel._validate_environment()
        if not left_env:
            return

        right_env = self.right_panel._validate_environment()
        if not right_env:
            return

        left_resolved = self.left_panel._resolve_stored_program()
        if not left_resolved:
            return

        right_resolved = self.right_panel._resolve_stored_program()
        if not right_resolved:
            return

        left_rm, left_sdk = left_env
        right_rm, right_sdk = right_env

        left_slot, left_name = left_resolved
        right_slot, right_name = right_resolved

        left_text = self._dual_program_text(left_slot, left_name)
        right_text = self._dual_program_text(right_slot, right_name)

        ok = messagebox.askyesno(
            "Confirm Dual Arm movement",
            "BOTH ROBOT ARMS WILL MOVE AT THE SAME TIME.\n\n"
            f"LEFT  // 192.168.1.10\nProgram {left_text}\n\n"
            f"RIGHT // 192.168.1.103\nProgram {right_text}\n\n"
            "The two software launch threads will synchronize immediately "
            "before starting the controller commands.\n\n"
            "Confirm both work areas are clear and both hardware E-stops "
            "are accessible.\n\n"
            "START BOTH?",
            icon="warning",
            parent=self,
        )
        if not ok:
            return

        left_cmd = self.left_panel._base_command(left_rm, left_sdk) + [
            "--run",
            str(left_slot),
            "--side",
            "left",
            "--yes",
        ]
        right_cmd = self.right_panel._base_command(right_rm, right_sdk) + [
            "--run",
            str(right_slot),
            "--side",
            "right",
            "--yes",
        ]

        # Synchronize immediately before Popen in each panel worker.
        launch_barrier = threading.Barrier(2)

        self.dual_run_active = True
        self.dual_run_started_at = __import__("time").monotonic()
        self.dual_runtime_var.set("0.0 s")
        self.dual_status_var.set(
            f"DUAL RUN LAUNCHING // LEFT {left_slot} + RIGHT {right_slot}"
        )
        self.dual_run_btn.configure(state="disabled")
        self.dual_sync_btn.configure(state="disabled")
        self.dual_stop_btn.configure(state="normal")

        try:
            self.left_panel._start_process_synced(
                left_cmd,
                f"DUAL RUN // Program {left_text}",
                launch_barrier,
            )
            self.right_panel._start_process_synced(
                right_cmd,
                f"DUAL RUN // Program {right_text}",
                launch_barrier,
            )
        except Exception as exc:
            self.dual_status_var.set(
                f"FAULT // Could not start synchronized Dual Run: {exc}"
            )
            self.dual_run_active = False
            self.dual_run_btn.configure(state="normal")
            self.dual_sync_btn.configure(state="normal")
            self.dual_stop_btn.configure(state="disabled")

            # If one side already started its worker, stop it.
            self.left_panel._slow_stop()
            self.right_panel._slow_stop()
            return

        self._monitor_dual_run()

    def _monitor_dual_run(self):
        if not self.dual_run_active:
            return

        now = __import__("time").monotonic()
        if self.dual_run_started_at is not None:
            self.dual_runtime_var.set(
                f"{now - self.dual_run_started_at:.1f} s"
            )

        left_active = self.left_panel.has_active_command()
        right_active = self.right_panel.has_active_command()

        if left_active or right_active:
            state_bits = []
            state_bits.append("LEFT ACTIVE" if left_active else "LEFT COMPLETE")
            state_bits.append("RIGHT ACTIVE" if right_active else "RIGHT COMPLETE")
            self.dual_status_var.set(
                "DUAL RUN // " + " // ".join(state_bits)
            )
            self.dual_run_after_id = self.after(
                100,
                self._monitor_dual_run,
            )
            return

        elapsed = (
            now - self.dual_run_started_at
            if self.dual_run_started_at is not None
            else 0.0
        )
        self.dual_runtime_var.set(f"{elapsed:.1f} s")

        left_status = self.left_panel.status_var.get()
        right_status = self.right_panel.status_var.get()

        left_ok = left_status.startswith("COMPLETE //")
        right_ok = right_status.startswith("COMPLETE //")

        if left_ok and right_ok:
            self.dual_status_var.set(
                f"DUAL RUN COMPLETE // Both arms finished // {elapsed:.1f} s"
            )
        else:
            self.dual_status_var.set(
                "DUAL RUN FINISHED // Check LEFT and RIGHT status/telemetry"
            )

        self.dual_run_active = False
        self.dual_run_started_at = None
        self.dual_run_after_id = None
        self.dual_run_btn.configure(state="normal")
        self.dual_sync_btn.configure(state="normal")
        self.dual_stop_btn.configure(state="disabled")

    def _stop_both(self):
        # No confirmation: this mirrors the existing one-click SLOW STOP rule.
        self.dual_status_var.set(
            "STOP BOTH REQUESTED // Sending Slow Stop to LEFT + RIGHT"
        )

        self.left_panel._slow_stop()
        self.right_panel._slow_stop()

        self.dual_stop_btn.configure(state="disabled")

        # If STOP BOTH was pressed during a RUN BOTH .TXT motion, allow its
        # monitor to observe both subprocesses finishing and restore the UI.


    def _is_descendant(self, widget, ancestor):
        current = widget
        while current is not None:
            if current == ancestor:
                return True
            current = getattr(current, "master", None)
        return False

    def _dual_mousewheel(self, event):
        # v29 has no long per-arm page. Native Text/Listbox/Treeview widgets
        # handle their own scrolling, so no global page scroll is required.
        return


    def _path_row(self, parent, row, label, variable, browse_command):
        ttk.Label(
            parent,
            text=label,
            style="Panel.TLabel",
        ).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=5)

        entry = self._tech_entry(parent, textvariable=variable)
        entry.grid(
            row=row,
            column=1,
            columnspan=3,
            sticky="ew",
            pady=5,
            ipady=4,
        )

        ttk.Button(
            parent,
            text="Browse",
            command=browse_command,
        ).grid(row=row, column=4, padx=(8, 0), pady=5)

    def _browse_rm_upload(self):
        path = filedialog.askopenfilename(
            title="Select rm_upload.py",
            filetypes=[
                ("Python files", "*.py"),
                ("All files", "*.*"),
            ],
            parent=self,
        )
        if path:
            self.rm_upload_var.set(path)

    def _browse_sdk(self):
        path = filedialog.askdirectory(
            title="Select RealMan SDK directory",
            parent=self,
        )
        if path:
            self.sdk_var.set(path)

    def _on_close(self):
        active = []

        if self.left_panel and self.left_panel.has_active_command():
            active.append("LEFT arm (192.168.1.10)")

        if self.right_panel and self.right_panel.has_active_command():
            active.append("RIGHT arm (192.168.1.103)")

        if active:
            messagebox.showwarning(
                "Controller command active",
                "A command is still active on:\n\n"
                + "\n".join(f"• {name}" for name in active)
                + "\n\nWait for it to finish, or use that arm's SLOW STOP before closing.",
                parent=self,
            )
            return

        if self.dual_run_after_id is not None:
            try:
                self.after_cancel(self.dual_run_after_id)
            except Exception:
                pass
            self.dual_run_after_id = None

        if self.dual_drag_after_id is not None:
            try:
                self.after_cancel(self.dual_drag_after_id)
            except Exception:
                pass
            self.dual_drag_after_id = None

        if self.dual_drag_save_after_id is not None:
            try:
                self.after_cancel(self.dual_drag_save_after_id)
            except Exception:
                pass
            self.dual_drag_save_after_id = None

        if self.dual_file_run_after_id is not None:
            try:
                self.after_cancel(self.dual_file_run_after_id)
            except Exception:
                pass
            self.dual_file_run_after_id = None

        if self.page2_dual_after_id is not None:
            try:
                self.after_cancel(self.page2_dual_after_id)
            except Exception:
                pass
            self.page2_dual_after_id = None

        if self.dual_drag_save_dialog is not None:
            self._close_dual_drag_save_popup()

        if self.dual_file_run_dialog is not None:
            self._close_dual_file_run_popup()

        try:
            self.unbind_all("<MouseWheel>")
            self.unbind_all("<Button-4>")
            self.unbind_all("<Button-5>")
        except Exception:
            pass

        self.destroy()


if __name__ == "__main__":
    app = DualArmTrajectoryUI()
    app.mainloop()
