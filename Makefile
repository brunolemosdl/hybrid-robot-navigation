PYTHON = python3
SRC_DIR = src
SCENES_DIR = scenes
RESULTS_DIR = results
LOGS_DIR = logs

HOST ?= localhost
PORT ?= 23000
OUTPUT_DIR ?= ./results
SCENE_PATH ?= ./scenes

ROBOTS = differential holonomic
SCENES = scene_1 scene_2 scene_3 scene_4
ALGORITHMS = rrt roadmap wavefront potential_fields extract_map

GREEN = \033[32m
YELLOW = \033[33m
RED = \033[31m
BLUE = \033[34m
MAGENTA = \033[35m
CYAN = \033[36m
NC = \033[0m

# Planners
GLOBAL ?= wavefront   # planner global padrão
LOCAL  ?= dwa         # planner local (DWA por padrão)

EXTRA_FLAGS += --global $(GLOBAL) --local $(LOCAL)


ifeq ($(VISUALIZE),1)
	EXTRA_FLAGS += --visualize
endif

ifeq ($(NAVIGATE_ONLY),1)
	EXTRA_FLAGS += --navigate-only
endif

ifeq ($(VERBOSE),1)
	EXTRA_FLAGS += --verbose
endif

ifneq ($(OUTPUT_DIR),./results)
	EXTRA_FLAGS += --output-dir $(OUTPUT_DIR)
endif

ifneq ($(SCENE_PATH),./scenes)
	EXTRA_FLAGS += --scene-path $(SCENE_PATH)
endif

.PHONY: help all clean setup rrt roadmap wavefront potential_fields extract_map install status check-deps run-all test-connection benchmark compare-algorithms logs dev

all: help

help:
	@echo "$(CYAN)════════════════════════════════════════════════════════════════════════════════$(NC)"
	@echo "$(CYAN)              MOBILE ROBOTICS - PATH PLANNING AND NAVIGATION                    $(NC)"
	@echo "$(CYAN)════════════════════════════════════════════════════════════════════════════════$(NC)"
	@echo ""
	@echo "$(GREEN)USAGE:$(NC)"
	@echo "  make <algorithm> ROBOT=<type> SCENE=<scene> [OPTIONS]"
	@echo ""
		@echo "$(GREEN)PLANNERS:$(NC)"
	@echo "  $(BLUE)GLOBAL$(NC)            - Global planner: rrt | roadmap | wavefront | potential_fields (default: wavefront)"
	@echo "  $(BLUE)LOCAL$(NC)             - Local planner: dwa | none (default: dwa)"
	@echo ""
	@echo "$(GREEN)EXAMPLES (com DWA):$(NC)"
	@echo "  make wavefront ROBOT=differential SCENE=scene_1 LOCAL=dwa"
	@echo "  make rrt ROBOT=differential SCENE=scene_2 LOCAL=dwa VERBOSE=1"
	@echo "  make extract_map ROBOT=differential SCENE=scene_1   # (não usa local planner)"
	@echo "$(GREEN)ALGORITHMS:$(NC)"
	@echo "  $(YELLOW)rrt$(NC)               - Rapidly-exploring Random Tree algorithm"
	@echo "  $(YELLOW)roadmap$(NC)           - Roadmap algorithm"
	@echo "  $(YELLOW)wavefront$(NC)         - Wavefront algorithm"
	@echo "  $(YELLOW)potential_fields$(NC)  - Potential Fields algorithm"
	@echo "  $(YELLOW)extract_map$(NC)       - Extract map only (no path planning)"
	@echo ""
	@echo "$(GREEN)ROBOTS:$(NC) $(MAGENTA)differential$(NC) | $(MAGENTA)holonomic$(NC)"
	@echo "$(GREEN)SCENES:$(NC) $(MAGENTA)scene_1$(NC) | $(MAGENTA)scene_2$(NC)"
	@echo ""
	@echo "$(GREEN)OPTIONS:$(NC)"
	@echo "  $(BLUE)VISUALIZE=1$(NC)       - Enable visualization"
	@echo "  $(BLUE)VERBOSE=1$(NC)         - Enable verbose logging with detailed debug information"
	@echo "  $(BLUE)NAVIGATE_ONLY=1$(NC)   - Use saved path for navigation only"
	@echo "  $(BLUE)HOST$(NC)              - Host address (default: localhost)"
	@echo "  $(BLUE)PORT$(NC)              - Port number (default: 23000)"
	@echo "  $(BLUE)OUTPUT_DIR$(NC)        - Output directory (default: ./results)"
	@echo "  $(BLUE)SCENE_PATH$(NC)        - Scene files directory (default: ./scenes)"
	@echo ""
	@echo "$(GREEN)EXAMPLES:$(NC)"
	@echo "  make rrt ROBOT=differential SCENE=scene_1"
	@echo "  make roadmap ROBOT=holonomic SCENE=scene_2 VISUALIZE=1"
	@echo "  make extract_map ROBOT=differential SCENE=scene_1 VISUALIZE=1"
	@echo "  make rrt ROBOT=differential SCENE=scene_1 VERBOSE=1"
	@echo "  make wavefront ROBOT=holonomic SCENE=scene_2 VISUALIZE=1 VERBOSE=1"
	@echo "  make run-all ROBOT=differential SCENE=scene_1 VISUALIZE=1"
	@echo ""
	@echo "$(GREEN)UTILITY COMMANDS:$(NC)"
	@echo "  $(YELLOW)setup$(NC)             - Install dependencies and create directories"
	@echo "  $(YELLOW)check-deps$(NC)        - Check if all dependencies are installed"
	@echo "  $(YELLOW)test-connection$(NC)   - Test CoppeliaSim connection"
	@echo "  $(YELLOW)status$(NC)            - Show project status and recent results"
	@echo "  $(YELLOW)clean$(NC)             - Clean generated files"
	@echo "  $(YELLOW)logs$(NC)              - View recent log entries"
	@echo ""
	@echo "$(GREEN)ADVANCED COMMANDS:$(NC)"
	@echo "  $(YELLOW)run-all$(NC)           - Run all algorithms for given robot and scene"
	@echo "  $(YELLOW)benchmark$(NC)         - Benchmark all algorithms (requires ROBOT and SCENE)"
	@echo "  $(YELLOW)compare-algorithms$(NC) - Compare algorithm performance"
	@echo "  $(YELLOW)dev$(NC)               - Development setup with verbose logging"
	@echo ""
	@echo "$(CYAN)════════════════════════════════════════════════════════════════════════════════$(NC)"

validate-params:
	@if [ -z "$(ROBOT)" ]; then echo "$(RED)Error: ROBOT not specified$(NC)"; echo "Use: make <algorithm> ROBOT=<differential|holonomic> SCENE=<scene_1|scene_2|scene_3|scene_4>"; exit 1; fi
	@if [ -z "$(SCENE)" ]; then echo "$(RED)Error: SCENE not specified$(NC)"; echo "Use: make <algorithm> ROBOT=<differential|holonomic> SCENE=<scene_1|scene_2|scene_3|scene_4>"; exit 1; fi
	@if [ "$(ROBOT)" != "differential" ] && [ "$(ROBOT)" != "holonomic" ]; then echo "$(RED)Error: Invalid ROBOT '$(ROBOT)'$(NC)"; echo "Valid options: differential, holonomic"; exit 1; fi
	@if [ "$(SCENE)" != "scene_1" ] && [ "$(SCENE)" != "scene_2" ] && [ "$(SCENE)" != "scene_3" ] && [ "$(SCENE)" != "scene_4" ]; then echo "$(RED)Error: Invalid SCENE '$(SCENE)'$(NC)"; echo "Valid options: scene_1, scene_2, scene_3, scene_4"; exit 1; fi
	@echo "$(GREEN)✓ Parameters validated: ROBOT=$(ROBOT), SCENE=$(SCENE)$(NC)"

rrt: validate-params
	@echo "$(CYAN)Running RRT algorithm...$(NC)"
	$(PYTHON) $(SRC_DIR)/main.py rrt $(ROBOT) $(SCENE) --host $(HOST) --port $(PORT) $(EXTRA_FLAGS)

roadmap: validate-params
	@echo "$(CYAN)Running Roadmap algorithm...$(NC)"
	$(PYTHON) $(SRC_DIR)/main.py roadmap $(ROBOT) $(SCENE) --host $(HOST) --port $(PORT) $(EXTRA_FLAGS)

wavefront: validate-params
	@echo "$(CYAN)Running Wavefront algorithm...$(NC)"
	$(PYTHON) $(SRC_DIR)/main.py wavefront $(ROBOT) $(SCENE) --host $(HOST) --port $(PORT) $(EXTRA_FLAGS)

potential_fields: validate-params
	@echo "$(CYAN)Running Potential Fields algorithm...$(NC)"
	$(PYTHON) $(SRC_DIR)/main.py potential_fields $(ROBOT) $(SCENE) --host $(HOST) --port $(PORT) $(EXTRA_FLAGS)

extract_map: validate-params
	@echo "$(CYAN)Extracting map from simulation...$(NC)"
	$(PYTHON) $(SRC_DIR)/main.py extract_map $(ROBOT) $(SCENE) --host $(HOST) --port $(PORT) $(EXTRA_FLAGS)

check-deps:
	@echo "$(CYAN)Checking dependencies...$(NC)"
	@$(PYTHON) -c "import sys; print(f'Python version: {sys.version}')"
	@$(PYTHON) -c "import numpy; print(f'NumPy version: {numpy.__version__}')" 2>/dev/null || echo "$(RED)✗ NumPy not installed$(NC)"
	@$(PYTHON) -c "import cv2; print(f'OpenCV version: {cv2.__version__}')" 2>/dev/null || echo "$(RED)✗ OpenCV not installed$(NC)"
	@$(PYTHON) -c "import matplotlib; print(f'Matplotlib version: {matplotlib.__version__}')" 2>/dev/null || echo "$(RED)✗ Matplotlib not installed$(NC)"
	@$(PYTHON) -c "import scipy; print(f'SciPy version: {scipy.__version__}')" 2>/dev/null || echo "$(RED)✗ SciPy not installed$(NC)"
	@$(PYTHON) -c "import networkx; print(f'NetworkX version: {networkx.__version__}')" 2>/dev/null || echo "$(RED)✗ NetworkX not installed$(NC)"
	@$(PYTHON) -c "import colorlog; print(f'ColorLog version: {colorlog.__version__}')" 2>/dev/null || echo "$(RED)✗ ColorLog not installed$(NC)"
	@echo "$(GREEN)✓ Dependency check completed$(NC)"

test-connection:
	@echo "$(CYAN)Testing CoppeliaSim connection...$(NC)"
	@$(PYTHON) -c "import socket; socket.create_connection(('$(HOST)', $(PORT)), timeout=2); print('$(GREEN)✓ Connection successful$(NC)')" 2>/dev/null || echo "$(RED)✗ Could not connect to CoppeliaSim at $(HOST):$(PORT)$(NC)"

status:
	@echo "$(CYAN)Project Status:$(NC)"
	@echo "$(YELLOW)Configuration:$(NC)"
	@echo "  Host: $(HOST):$(PORT)"
	@echo "  Scenes directory: $(SCENE_PATH)"
	@echo "  Output directory: $(OUTPUT_DIR)"
	@echo ""
	@echo "$(YELLOW)Recent results:$(NC)"
	@if [ -d "$(RESULTS_DIR)" ]; then \
		find $(RESULTS_DIR) -name "*.json" -type f -exec ls -lt {} + 2>/dev/null | head -5 || echo "  No results found"; \
	else \
		echo "  Results directory not found"; \
	fi
	@echo ""
	@echo "$(YELLOW)Log files:$(NC)"
	@if [ -d "$(LOGS_DIR)" ]; then \
		find $(LOGS_DIR) -name "*.log" -type f -exec ls -lt {} + 2>/dev/null | head -3 || echo "  No log files found"; \
	else \
		echo "  Logs directory not found"; \
	fi

logs:
	@echo "$(CYAN)Recent log entries:$(NC)"
	@if [ -d "$(LOGS_DIR)" ] && [ -n "$$(find $(LOGS_DIR) -name 'robotics_session_*.log' 2>/dev/null)" ]; then \
		latest_log=$$(find $(LOGS_DIR) -name 'robotics_session_*.log' -type f -exec ls -t {} + 2>/dev/null | head -1); \
		echo "$(YELLOW)Showing latest log file: $$(basename $$latest_log)$(NC)"; \
		tail -20 $$latest_log; \
	else \
		echo "$(YELLOW)No log files found. Run a command to generate logs.$(NC)"; \
	fi

run-all: validate-params
	@echo "$(CYAN)Running all algorithms for $(ROBOT) robot on $(SCENE)...$(NC)"
	@echo "$(YELLOW)1/4 - RRT Algorithm$(NC)"
	@$(MAKE) rrt ROBOT=$(ROBOT) SCENE=$(SCENE) VERBOSE=$(VERBOSE) VISUALIZE=$(VISUALIZE) || true
	@echo "$(YELLOW)2/4 - Roadmap Algorithm$(NC)"
	@$(MAKE) roadmap ROBOT=$(ROBOT) SCENE=$(SCENE) VERBOSE=$(VERBOSE) VISUALIZE=$(VISUALIZE) || true
	@echo "$(YELLOW)3/4 - Wavefront Algorithm$(NC)"
	@$(MAKE) wavefront ROBOT=$(ROBOT) SCENE=$(SCENE) VERBOSE=$(VERBOSE) VISUALIZE=$(VISUALIZE) || true
	@echo "$(YELLOW)4/4 - Potential Fields Algorithm$(NC)"
	@$(MAKE) potential_fields ROBOT=$(ROBOT) SCENE=$(SCENE) VERBOSE=$(VERBOSE) VISUALIZE=$(VISUALIZE) || true
	@echo "$(GREEN)✓ All algorithms completed$(NC)"

benchmark: validate-params
	@echo "$(CYAN)Benchmarking all algorithms...$(NC)"
	@echo "$(YELLOW)Running benchmark for $(ROBOT) robot on $(SCENE)$(NC)"
	@echo "Algorithm,Robot,Scene,Success,Time" > benchmark_results.csv
	@for alg in $(filter-out extract_map,$(ALGORITHMS)); do \
		echo "$(CYAN)Benchmarking $$alg...$(NC)"; \
		start_time=$$(date +%s); \
		if $(MAKE) $$alg ROBOT=$(ROBOT) SCENE=$(SCENE) >/dev/null 2>&1; then \
			end_time=$$(date +%s); \
			duration=$$((end_time - start_time)); \
			echo "$$alg,$(ROBOT),$(SCENE),Success,$$duration" >> benchmark_results.csv; \
			echo "$(GREEN)✓ $$alg completed in $${duration}s$(NC)"; \
		else \
			echo "$$alg,$(ROBOT),$(SCENE),Failed,0" >> benchmark_results.csv; \
			echo "$(RED)✗ $$alg failed$(NC)"; \
		fi; \
	done
	@echo "$(GREEN)✓ Benchmark results saved to benchmark_results.csv$(NC)"

compare-algorithms:
	@echo "$(CYAN)Comparing algorithm results...$(NC)"
	@if [ ! -f "benchmark_results.csv" ]; then \
		echo "$(RED)No benchmark results found. Run 'make benchmark' first.$(NC)"; \
		exit 1; \
	fi
	@echo "$(YELLOW)Benchmark Results:$(NC)"
	@column -t -s',' benchmark_results.csv 2>/dev/null || cat benchmark_results.csv

dev: 
	@echo "$(CYAN)Setting up development environment...$(NC)"
	@$(MAKE) setup
	@$(MAKE) check-deps
	@$(MAKE) test-connection
	@echo "$(GREEN)✓ Development environment ready$(NC)"
	@echo "$(YELLOW)Use VERBOSE=1 for detailed logging during development$(NC)"

install:
	@echo "$(CYAN)Installing dependencies...$(NC)"
	$(PYTHON) -m pip install -r requirements.txt
	@echo "$(GREEN)✓ Dependencies installed$(NC)"

setup: install
	@echo "$(CYAN)Setting up project directories...$(NC)"
	@mkdir -p $(RESULTS_DIR) $(LOGS_DIR)
	@echo "$(GREEN)✓ Project setup completed$(NC)"
	@echo "$(YELLOW)Directories created:$(NC)"
	@echo "  - $(RESULTS_DIR)/ (for algorithm results)"
	@echo "  - $(LOGS_DIR)/ (for log files)"

clean:
	@echo "$(CYAN)Cleaning generated files...$(NC)"
	@find . -type f -name "*.pyc" -delete
	@find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	@rm -rf $(RESULTS_DIR)/* $(LOGS_DIR)/* 2>/dev/null || true
	@rm -f benchmark_results.csv 2>/dev/null || true
	@echo "$(GREEN)✓ Clean completed!$(NC)"

setup:
	poetry install

format:
	ruff format .