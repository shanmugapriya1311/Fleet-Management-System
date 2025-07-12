import time
import logging
import threading
from pathlib import Path
from logging.handlers import RotatingFileHandler
from src.models.nav_graph import NavGraph
from controllers.fleet_manager import FleetManager
from gui.fleet_gui import FleetGUI

def setup_logging():
    """Setup logging configuration with rotation"""
    log_dir = Path('logs')
    log_dir.mkdir(exist_ok=True)
    
    # Create logger
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    
    # Create rotating file handler
    file_handler = RotatingFileHandler(
        'logs/fleet_logs.txt',
        maxBytes=10*1024*1024,  # 10MB
        backupCount=5
    )
    file_handler.setLevel(logging.INFO)
    
    # Create formatter
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)
    
    # Add handler to logger
    logger.addHandler(file_handler)

def simulation_thread(fleet_manager, gui):
    """Enhanced simulation thread with GUI updates"""
    try:
        while True:
            fleet_manager.update_robots()
            # Update GUI's robot states
            gui.robots = fleet_manager.robots
            time.sleep(0.1)
    except Exception as e:
        logging.error(f"Error in simulation thread: {e}")
        gui.add_alert(f"Simulation error: {str(e)}")

def load_nav_graph():
    """Load navigation graph with error handling"""
    while True:
        print("\nChoose navigation graph:")
        print("1. nav_graph_1.json")
        print("2. nav_graph_2.json")
        print("3. nav_graph_3.json")
        
        try:
            choice = input("Enter number (1-3): ")
            if choice not in ['1', '2', '3']:
                print("Invalid choice. Please enter 1, 2, or 3.")
                continue
                
            graph_file = f"data/nav_graph_{choice}.json"
            nav_graph = NavGraph()
            nav_graph.load_from_json(graph_file)
            logging.info(f"Successfully loaded navigation graph: {graph_file}")
            return nav_graph
            
        except FileNotFoundError:
            print(f"Error: Could not find {graph_file}")
            logging.error(f"Navigation graph file not found: {graph_file}")
        except Exception as e:
            print(f"Error loading navigation graph: {e}")
            logging.error(f"Error loading navigation graph: {e}")

def main():
    try:
        # Setup logging
        setup_logging()
        logging.info("Starting Fleet Management System")

        # Initialize navigation graph
        nav_graph = load_nav_graph()
        if not nav_graph:
            print("Failed to load navigation graph. Exiting.")
            return

        # Initialize fleet manager
        fleet_manager = FleetManager(nav_graph)
        
        # Initialize GUI with fleet manager reference
        gui = FleetGUI(nav_graph, fleet_manager)
        
        # Create and start simulation thread
        sim_thread = threading.Thread(
            target=simulation_thread,
            args=(fleet_manager, gui),
            daemon=True
        )
        sim_thread.start()
        
        # Run GUI
        gui.run()
        
    except Exception as e:
        logging.error(f"Critical error in main: {e}")
        print(f"Critical error: {e}")
    finally:
        logging.info("Shutting down Fleet Management System")

if __name__ == "__main__":
    main()