setup:
	poetry install

run-drive:
	poetry run python -m src.scripts.drive_epuck

format:
	ruff format .
