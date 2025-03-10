import tkinter as tk
from tkinter import messagebox

def get_parameters():
    root = tk.Tk()
    root.title("Parameter Input")

    # Track Distance
    tk.Label(root, text="Track Distance:").grid(row=0, column=0, padx=10, pady=10)
    track_distance_entry = tk.Entry(root)
    track_distance_entry.grid(row=0, column=1, padx=10, pady=10)
    track_distance_entry.insert(0, "5.0")

    # Safe Distance
    tk.Label(root, text="Safe Distance:").grid(row=1, column=0, padx=10, pady=10)
    safe_distance_entry = tk.Entry(root)
    safe_distance_entry.grid(row=1, column=1, padx=10, pady=10)
    safe_distance_entry.insert(0, "3.0")

    # V_MAX
    tk.Label(root, text="V_MAX:").grid(row=2, column=0, padx=10, pady=10)
    v_max_entry = tk.Entry(root)
    v_max_entry.grid(row=2, column=1, padx=10, pady=10)
    v_max_entry.insert(0, "1.5")

    params = {}

    def submit():
        try:
            params['track_distance'] = float(track_distance_entry.get())
            params['safe_distance'] = float(safe_distance_entry.get())
            params['v_max'] = float(v_max_entry.get())
            root.quit()
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numeric values.")

    tk.Button(root, text="Launch", command=submit).grid(row=3, column=0, columnspan=2, pady=10)

    root.mainloop()
    root.destroy()

    return params

if __name__ == '__main__':
    params = get_parameters()
    print(f"track_distance={params['track_distance']}")
    print(f"safe_distance={params['safe_distance']}")
    print(f"v_max={params['v_max']}")
