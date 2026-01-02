import tkinter as tk
from tkinter import messagebox, ttk
import json
import hashlib
from datetime import datetime

class TodoApp:
    def __init__(self):
        self.users_file = "users.json"
        self.current_user = None
        self.load_users()
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("To-Do List Manager")
        self.root.geometry("500x600")
        self.root.resizable(False, False)
        
        self.show_login_screen()
    
    def load_users(self):
        """Load users from file"""
        try:
            with open(self.users_file, 'r') as f:
                self.users = json.load(f)
        except FileNotFoundError:
            self.users = {}
    
    def save_users(self):
        """Save users to file"""
        with open(self.users_file, 'w') as f:
            json.dump(self.users, f, indent=4)
    
    def hash_password(self, password):
        """Hash password for security"""
        return hashlib.sha256(password.encode()).hexdigest()
    
    def clear_window(self):
        """Clear all widgets from window"""
        for widget in self.root.winfo_children():
            widget.destroy()
    
    def show_login_screen(self):
        """Display login/register screen"""
        self.clear_window()
        
        # Title
        title = tk.Label(self.root, text="📝 To-Do List Manager", 
                        font=("Arial", 24, "bold"), fg="#2c3e50")
        title.pack(pady=30)
        
        # Login Frame
        login_frame = tk.Frame(self.root, bg="#ecf0f1", padx=40, pady=30)
        login_frame.pack(pady=20)
        
        tk.Label(login_frame, text="Username:", font=("Arial", 12), 
                bg="#ecf0f1").grid(row=0, column=0, sticky="w", pady=10)
        self.username_entry = tk.Entry(login_frame, font=("Arial", 12), width=25)
        self.username_entry.grid(row=0, column=1, pady=10)
        
        tk.Label(login_frame, text="Password:", font=("Arial", 12), 
                bg="#ecf0f1").grid(row=1, column=0, sticky="w", pady=10)
        self.password_entry = tk.Entry(login_frame, font=("Arial", 12), 
                                      width=25, show="*")
        self.password_entry.grid(row=1, column=1, pady=10)
        
        # Buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=20)
        
        login_btn = tk.Button(button_frame, text="Login", font=("Arial", 12, "bold"),
                             bg="#3498db", fg="white", width=12, command=self.login)
        login_btn.grid(row=0, column=0, padx=10)
        
        register_btn = tk.Button(button_frame, text="Register", font=("Arial", 12, "bold"),
                                bg="#2ecc71", fg="white", width=12, command=self.register)
        register_btn.grid(row=0, column=1, padx=10)
        
        # Bind Enter key
        self.password_entry.bind('<Return>', lambda e: self.login())
    
    def login(self):
        """Handle login"""
        username = self.username_entry.get().strip()
        password = self.password_entry.get()
        
        if not username or not password:
            messagebox.showerror("Error", "Please fill in all fields!")
            return
        
        if username in self.users:
            if self.users[username]['password'] == self.hash_password(password):
                self.current_user = username
                messagebox.showinfo("Success", f"Welcome back, {username}!")
                self.show_todo_screen()
            else:
                messagebox.showerror("Error", "Incorrect password!")
        else:
            messagebox.showerror("Error", "User not found! Please register.")
    
    def register(self):
        """Handle registration"""
        username = self.username_entry.get().strip()
        password = self.password_entry.get()
        
        if not username or not password:
            messagebox.showerror("Error", "Please fill in all fields!")
            return
        
        if len(password) < 4:
            messagebox.showerror("Error", "Password must be at least 4 characters!")
            return
        
        if username in self.users:
            messagebox.showerror("Error", "Username already exists!")
            return
        
        self.users[username] = {
            'password': self.hash_password(password),
            'tasks': []
        }
        self.save_users()
        messagebox.showinfo("Success", "Account created! Please login.")
        self.username_entry.delete(0, tk.END)
        self.password_entry.delete(0, tk.END)
    
    def show_todo_screen(self):
        """Display main to-do list screen"""
        self.clear_window()
        
        # Header
        header = tk.Frame(self.root, bg="#3498db", height=60)
        header.pack(fill="x")
        
        tk.Label(header, text=f"Welcome, {self.current_user}!", 
                font=("Arial", 16, "bold"), bg="#3498db", fg="white").pack(side="left", padx=20, pady=15)
        
        logout_btn = tk.Button(header, text="Logout", font=("Arial", 10),
                              bg="#e74c3c", fg="white", command=self.logout)
        logout_btn.pack(side="right", padx=20)
        
        # Add task section
        add_frame = tk.Frame(self.root, bg="#ecf0f1", pady=20)
        add_frame.pack(fill="x", padx=20, pady=10)
        
        self.task_entry = tk.Entry(add_frame, font=("Arial", 12), width=35)
        self.task_entry.pack(side="left", padx=10)
        
        add_btn = tk.Button(add_frame, text="Add Task", font=("Arial", 11, "bold"),
                           bg="#2ecc71", fg="white", command=self.add_task)
        add_btn.pack(side="left")
        
        # Task list
        list_frame = tk.Frame(self.root)
        list_frame.pack(fill="both", expand=True, padx=20, pady=10)
        
        # Scrollbar
        scrollbar = tk.Scrollbar(list_frame)
        scrollbar.pack(side="right", fill="y")
        
        self.task_listbox = tk.Listbox(list_frame, font=("Arial", 11),
                                       yscrollcommand=scrollbar.set, height=15)
        self.task_listbox.pack(side="left", fill="both", expand=True)
        scrollbar.config(command=self.task_listbox.yview)
        
        # Buttons
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=10)
        
        complete_btn = tk.Button(btn_frame, text="✓ Complete", font=("Arial", 10),
                                bg="#f39c12", fg="white", width=12, 
                                command=self.complete_task)
        complete_btn.grid(row=0, column=0, padx=5)
        
        delete_btn = tk.Button(btn_frame, text="✗ Delete", font=("Arial", 10),
                              bg="#e74c3c", fg="white", width=12,
                              command=self.delete_task)
        delete_btn.grid(row=0, column=1, padx=5)
        
        clear_btn = tk.Button(btn_frame, text="Clear All", font=("Arial", 10),
                             bg="#95a5a6", fg="white", width=12,
                             command=self.clear_completed)
        clear_btn.grid(row=0, column=2, padx=5)
        
        # Bind Enter key
        self.task_entry.bind('<Return>', lambda e: self.add_task())
        
        # Load tasks
        self.refresh_tasks()
    
    def add_task(self):
        """Add a new task"""
        task_text = self.task_entry.get().strip()
        
        if not task_text:
            messagebox.showwarning("Warning", "Please enter a task!")
            return
        
        task = {
            'description': task_text,
            'completed': False,
            'created_at': datetime.now().strftime("%Y-%m-%d %H:%M")
        }
        
        self.users[self.current_user]['tasks'].append(task)
        self.save_users()
        self.task_entry.delete(0, tk.END)
        self.refresh_tasks()
    
    def refresh_tasks(self):
        """Refresh the task list display"""
        self.task_listbox.delete(0, tk.END)
        
        tasks = self.users[self.current_user]['tasks']
        for i, task in enumerate(tasks):
            status = "✓" if task['completed'] else "○"
            display_text = f"{status} {task['description']} ({task['created_at']})"
            self.task_listbox.insert(tk.END, display_text)
            
            if task['completed']:
                self.task_listbox.itemconfig(i, fg="gray")
    
    def complete_task(self):
        """Mark selected task as complete"""
        try:
            index = self.task_listbox.curselection()[0]
            self.users[self.current_user]['tasks'][index]['completed'] = True
            self.save_users()
            self.refresh_tasks()
        except IndexError:
            messagebox.showwarning("Warning", "Please select a task!")
    
    def delete_task(self):
        """Delete selected task"""
        try:
            index = self.task_listbox.curselection()[0]
            self.users[self.current_user]['tasks'].pop(index)
            self.save_users()
            self.refresh_tasks()
        except IndexError:
            messagebox.showwarning("Warning", "Please select a task!")
    
    def clear_completed(self):
        """Clear all completed tasks"""
        tasks = self.users[self.current_user]['tasks']
        self.users[self.current_user]['tasks'] = [t for t in tasks if not t['completed']]
        self.save_users()
        self.refresh_tasks()
        messagebox.showinfo("Success", "Completed tasks cleared!")
    
    def logout(self):
        """Logout and return to login screen"""
        self.current_user = None
        self.show_login_screen()
    
    def run(self):
        """Start the application"""
        self.root.mainloop()


if __name__ == "__main__":
    app = TodoApp()
    app.run()