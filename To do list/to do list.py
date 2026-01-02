import json
import os
from datetime import datetime

class TodoList:
    def __init__(self, filename="tasks.json"):
        self.filename = filename
        self.tasks = self.load_tasks()
    
    def load_tasks(self):
        """Load tasks from JSON file"""
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r') as f:
                    return json.load(f)
            except json.JSONDecodeError:
                return []
        return []
    
    def save_tasks(self):
        """Save tasks to JSON file"""
        with open(self.filename, 'w') as f:
            json.dump(self.tasks, f, indent=4)
    
    def add_task(self, task_description):
        """Add a new task"""
        task = {
            'id': len(self.tasks) + 1,
            'description': task_description,
            'completed': False,
            'created_at': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        self.tasks.append(task)
        self.save_tasks()
        print(f"✓ Task added: '{task_description}'")
    
    def view_tasks(self):
        """Display all tasks"""
        if not self.tasks:
            print("\n📝 No tasks yet! Add one to get started.")
            return
        
        print("\n" + "="*60)
        print("YOUR TO-DO LIST".center(60))
        print("="*60)
        
        for task in self.tasks:
            status = "✓" if task['completed'] else "○"
            task_text = task['description']
            if task['completed']:
                task_text = f"\033[9m{task_text}\033[0m"  # Strikethrough
            
            print(f"{status} [{task['id']}] {task_text}")
            print(f"   Created: {task['created_at']}")
            print("-" * 60)
    
    def complete_task(self, task_id):
        """Mark a task as completed"""
        for task in self.tasks:
            if task['id'] == task_id:
                task['completed'] = True
                self.save_tasks()
                print(f"✓ Task {task_id} marked as completed!")
                return
        print(f"✗ Task {task_id} not found.")
    
    def delete_task(self, task_id):
        """Delete a task"""
        for i, task in enumerate(self.tasks):
            if task['id'] == task_id:
                deleted_task = self.tasks.pop(i)
                # Reorder IDs
                for j in range(i, len(self.tasks)):
                    self.tasks[j]['id'] = j + 1
                self.save_tasks()
                print(f"✓ Task deleted: '{deleted_task['description']}'")
                return
        print(f"✗ Task {task_id} not found.")
    
    def clear_completed(self):
        """Remove all completed tasks"""
        original_count = len(self.tasks)
        self.tasks = [task for task in self.tasks if not task['completed']]
        # Reorder IDs
        for i, task in enumerate(self.tasks):
            task['id'] = i + 1
        self.save_tasks()
        cleared = original_count - len(self.tasks)
        print(f"✓ Cleared {cleared} completed task(s).")


def display_menu():
    """Display the main menu"""
    print("\n" + "="*60)
    print("TO-DO LIST MENU".center(60))
    print("="*60)
    print("1. View all tasks")
    print("2. Add a new task")
    print("3. Complete a task")
    print("4. Delete a task")
    print("5. Clear completed tasks")
    print("6. Exit")
    print("="*60)


def main():
    todo = TodoList()
    
    print("\n🎯 Welcome to Your To-Do List Manager!")
    
    while True:
        display_menu()
        choice = input("\nEnter your choice (1-6): ").strip()
        
        if choice == '1':
            todo.view_tasks()
        
        elif choice == '2':
            task = input("\nEnter task description: ").strip()
            if task:
                todo.add_task(task)
            else:
                print("✗ Task description cannot be empty!")
        
        elif choice == '3':
            todo.view_tasks()
            try:
                task_id = int(input("\nEnter task ID to complete: "))
                todo.complete_task(task_id)
            except ValueError:
                print("✗ Please enter a valid number!")
        
        elif choice == '4':
            todo.view_tasks()
            try:
                task_id = int(input("\nEnter task ID to delete: "))
                todo.delete_task(task_id)
            except ValueError:
                print("✗ Please enter a valid number!")
        
        elif choice == '5':
            todo.clear_completed()
        
        elif choice == '6':
            print("\n👋 Goodbye! Your tasks have been saved.")
            break
        
        else:
            print("✗ Invalid choice! Please enter a number between 1-6.")


if __name__ == "__main__":
    main()