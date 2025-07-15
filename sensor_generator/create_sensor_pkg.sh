#!/bin/bash
# ===========================
# Script: create_sensor_pkg.sh
# Description: Creates a sensor plugin package with launch and model templates
# Author: Sebastián Tilaguy
# ===========================

# --- Source helper functions ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/utils.sh"

# --- Validate input ---
validate_inputs "$1"

# --- Setup variables ---
setup_variables "$1" "$2"

# --- Create directory structure ---
log_info "Creating directory structure..."
create_directories

# --- Copy template files ---
log_info "Copying templates..."
copy_templates

# --- Replace placeholders ---
log_info "Replacing placeholders..."
replace_tokens

# --- Add sensor to all_sensors_world placeholders ---
log_info "Adding to all sensor test world..."
add_sensor_to_world

# --- Done ---
final_message
