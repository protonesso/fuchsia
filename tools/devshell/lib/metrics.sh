#!/bin/bash
# Copyright 2019 The Fuchsia Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

### common methods for metrics collection
### this script assumes that vars.sh has already been sourced, since it
### depends on FUCHSIA_DIR being defined correctly.

declare -r GA_PROPERTY_ID="UA-127897021-6"
declare -r TRACK_ALL_ARGS="set,fidlcat"
declare -r TRACK_RESULTS="set,build"
declare -r DEBUG_LOG_CONFIG="/tmp/.fx_metrics_debugfile"

# To properly enable unit testing, METRICS_CONFIG is not read-only
METRICS_CONFIG="${FUCHSIA_DIR}/.fx-metrics-config"

_METRICS_DEBUG=0
_METRICS_DEBUG_LOG_FILE=""
_METRICS_USE_VALIDATION_SERVER=0

INIT_WARNING=$'Please opt in or out of fx metrics collection.\n'
INIT_WARNING+=$'You will receive this warning until an option is selected.\n'
INIT_WARNING+=$'To check what data we collect, run `fx metrics`\n'
INIT_WARNING+=$'To opt in or out, run `fx metrics <enable|disable>\n'

# Each Analytics batch call can send at most this many hits.
declare -r BATCH_SIZE=20
# Keep track of how many hits have accumulated.
hit_count=0
# Holds curl args for the current batch of hits.
curl_args=()

function metrics-read-config {
  METRICS_UUID=""
  METRICS_ENABLED=0
  if [[ ! -f "${METRICS_CONFIG}" ]]; then
    return 1
  fi
  source "${METRICS_CONFIG}"
  if [[ $METRICS_ENABLED == 1 && -z "$METRICS_UUID" ]]; then
    METRICS_ENABLED=0
    return 1
  fi
  return 0
}

function metrics-write-config {
  enabled=$1
  if [[ "$enabled" -eq "1" ]]; then
    uuid="$2"
  fi
  local -r tempfile="$(mktemp)"

  # Exit trap to clean up temp file
  trap "[[ -f \"${tempfile}\" ]] && rm -f \"${tempfile}\"" EXIT

  {
    echo "# Autogenerated config file for fx metrics. Run 'fx help metrics' for more information."
    echo "METRICS_ENABLED=${enabled}"
    echo "METRICS_UUID=\"${uuid}\""
  } >> "${tempfile}"
  # Only rewrite the config file if content has changed
  if ! cmp --silent "${tempfile}" "${METRICS_CONFIG}" ; then
    mv -f "${tempfile}" "${METRICS_CONFIG}"
  fi
}

function metrics-read-and-validate {
  local hide_init_warning=$1
  if ! metrics-read-config; then
    if [[ $hide_init_warning -ne 1 ]]; then
      fx-warn "${INIT_WARNING}"
    fi
    return 1
  fi
  return 0
}

function metrics-set-debug-logfile {
  debug_logfile="$1"
  # make sure that either DEBUG_LOG_CONFIG is writable or it doesn't exist
  # and its directory is writable
  if [[ -w "${DEBUG_LOG_CONFIG}" || \
        ( ! -f "${DEBUG_LOG_CONFIG}" &&
          -w "$(dirname "${DEBUG_LOG_CONFIG}")" ) ]]; then
    echo "$debug_logfile" > "$DEBUG_LOG_CONFIG"
    return 0
  else
    fx-warn "Cannot persist the metrics log filename to ${DEBUG_LOG_CONFIG}"
    fx-warn "Ignoring debug logging of metrics collection"
  fi
  return 1
}

function metrics-get-debug-logfile {
  if [[ $_METRICS_DEBUG == 1 ]]; then
    echo "$_METRICS_DEBUG_LOG_FILE"
  elif [[ -f "$DEBUG_LOG_CONFIG" ]]; then
    head -1 "$DEBUG_LOG_CONFIG"
  fi
}

function metrics-maybe-log {
  local filename="$(metrics-get-debug-logfile)"
  if [[ $filename ]]; then
    if [[ ! -f "$filename" && -w $(dirname "$filename") ]]; then
      touch "$filename"
    fi
    if [[ -w "$filename" ]]; then
      TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
      echo -n "${TIMESTAMP}:" >> "$filename"
      for i in "$@"; do
        if [[ "$i" =~ ^"--" ]]; then
          continue # Skip switches.
        fi
        # Space before $i is intentional.
        echo -n " $i" >> "$filename"
      done
      # Add a newline at the end.
      echo >> "$filename"
    fi
  fi
}

# Arguments:
#   - the name of the fx subcommand
#   - args of the subcommand
function track-command-execution {
  subcommand="$1"
  shift
  args="$*"

  local hide_init_warning=0
  if [[ "$subcommand" == "metrics" ]]; then
    hide_init_warning=1
  fi
  metrics-read-and-validate $hide_init_warning
  if [[ $METRICS_ENABLED == 0 ]]; then
    return 0
  fi

  if [[ "$subcommand" == "set" ]]; then
    # Add separate fx_set hits for packages
    _process-fx-set-command "$@"
  fi

  # Only track arguments to the subcommands in $TRACK_ALL_ARGS
  if [[ $TRACK_ALL_ARGS != *"$subcommand"* ]]; then
    args=""
  else
    # Limit to the first 100 characters of arguments.
    # The Analytics API supports up to 500 bytes, but it is likely that
    # anything larger than 100 characters is an invalid execution and/or not
    # what we want to track.
    args=${args:0:100}
  fi

  analytics_args=(
    "t=event" \
    "ec=fx" \
    "ea=${subcommand}" \
    "el=${args}" \
    )

  _add-to-analytics-batch "${analytics_args[@]}"
  # Send any remaining hits.
  _send-analytics-batch
  return 0
}

# Arguments:
#   - args of `fx set`
function _process-fx-set-command {
  target="$1"
  shift
  while [[ $# -ne 0 ]]; do
    case $1 in
      --with)
        shift # remove "--with"
        _add-fx-set-hit "$target" "fx-with" "$1"
        ;;
      --with-base)
        shift # remove "--with-base"
        _add-fx-set-hit "$target" "fx-with-base" "$1"
        ;;
      *)
        ;;
    esac
    shift
  done
}

# Arguments:
#   - the product.board target for `fx set`
#   - category name, either "fx-with" or "fx-with-base"
#   - package(s) following "--with" or "--with-base" switch
function _add-fx-set-hit {
  target="$1"
  category="$2"
  packages="$3"
  # Packages argument can be a comma-separated list.
  IFS=',' read -ra packages_parts <<< "$packages"
  for p in "${packages_parts[@]}"; do
    analytics_args=(
      "t=event" \
      "ec=${category}" \
      "ea=${p}" \
      "el=${target}" \
      )

    _add-to-analytics-batch "${analytics_args[@]}"
  done
}

# Arguments:
#   - time taken to complete (milliseconds)
#   - exit status
#   - the name of the fx subcommand
#   - args of the subcommand
function track-command-finished {
  timing=$1
  exit_status=$2
  subcommand=$3
  shift 3
  args="$*"

  metrics-read-config
  if [[ $METRICS_ENABLED == 0 || $TRACK_RESULTS != *"$subcommand"* ]]; then
    return 0
  fi

  # Only track arguments to the subcommands in $TRACK_ALL_ARGS
  if [[ $TRACK_ALL_ARGS != *"$subcommand"* ]]; then
    args=""
  else
    # Limit to the first 100 characters of arguments.
    # The Analytics API supports up to 500 bytes, but it is likely that
    # anything larger than 100 characters is an invalid execution and/or not
    # what we want to track.
    args=${args:0:100}
  fi

  if [[ $exit_status == 0 ]]; then
    # Successes are logged as timing hits
    hit_type="timing"
    analytics_args=(
      "t=timing" \
      "utc=fx" \
      "utv=${subcommand}" \
      "utt=${timing}" \
      "utl=${args}" \
      )
  else
    # Failures are logged as event hits with a separate category
    # exit status is stored as Custom Dimension 1
    hit_type="event"
    analytics_args=(
      "t=event" \
      "ec=fx_exception" \
      "ea=${subcommand}" \
      "el=${args}" \
      "cd1=${exit_status}" \
      )
  fi

  _add-to-analytics-batch "${analytics_args[@]}"
  # Send any remaining hits.
  _send-analytics-batch
  return 0
}

# Add an analytics hit with the given args to the batch of hits. This will trigger
# sending a batch when the batch size limit is hit.
#
# Arguments:
#   - analytics arguments, e.g. "t=event" "ec=fx" etc.
function _add-to-analytics-batch {
  if [[ $# -eq 0 ]]; then
    return 0
  fi

  if (( hit_count > 0 )); then
    # Each hit in a batch must be on its own line. The below will append a newline
    # without url-encoding it. Note that this does add a '&' to the end of each hit,
    # but those are ignored by Google Analytics.
    curl_args+=(--data-binary)
    curl_args+=($'\n')
  fi

  # All hits send some common parameters
  local app_name="$(_app_name)"
  local app_version="$(_app_version)"
  params=(
    "v=1" \
    "tid=${GA_PROPERTY_ID}" \
    "cid=${METRICS_UUID}" \
    "an=${app_name}" \
    "av=${app_version}" \
    "$@" \
    )
  for p in "${params[@]}"; do
    curl_args+=(--data-urlencode)
    curl_args+=("$p")
  done

  : $(( hit_count += 1 ))
  if ((hit_count == BATCH_SIZE)); then
    _send-analytics-batch
  fi
}

# Sends the current batch of hits to the Analytics server. As a side effect, clears
# the hit count and batch data.
function _send-analytics-batch {
  if [[ $hit_count -eq 0 ]]; then
    return 0
  fi

  local user_agent="Fuchsia-fx $(_os_data)"
  local url_path="/batch"
  local result=""
  if [[ $_METRICS_DEBUG == 1 && $_METRICS_USE_VALIDATION_SERVER == 1 ]]; then
    url_path="/debug/collect"
    # Validation server does not accept batches. Send just the first hit instead.
    local limit=0
    for i in "${curl_args[@]}"; do
      if [[ "$i" == "--data-binary" ]]; then
        curl_args=("${curl_args[@]:0:$limit}")
        break
      fi
      : $(( limit += 1 ))
    done
  fi
  if [[ $_METRICS_DEBUG == 1 && $_METRICS_USE_VALIDATION_SERVER == 0 ]]; then
    # if testing and not using the validation server, always return 202
    result="202"
  elif [[ $_METRICS_DEBUG == 0 || $_METRICS_USE_VALIDATION_SERVER == 1 ]]; then
    result=$(curl -s -o /dev/null -w "%{http_code}" "${curl_args[@]}" \
      -H "User-Agent: $user_agent" \
      "https://www.google-analytics.com/${url_path}")
  fi
  metrics-maybe-log "${curl_args[@]}" "RESULT=${result}"

  # Clear batch.
  hit_count=0
  curl_args=()
}

function _os_data {
  if command -v uname >/dev/null 2>&1 ; then
    uname -rs
  else
    echo "Unknown"
  fi
}

function _app_name {
  if [[ -n "${BASH_VERSION}" ]]; then
    echo "bash"
  elif [[ -n "${ZSH_VERSION}" ]]; then
    echo "zsh"
  else
    echo "Unknown"
  fi
}

function _app_version {
  if [[ -n "${BASH_VERSION}" ]]; then
    echo "${BASH_VERSION}"
  elif [[ -n "${ZSH_VERSION}" ]]; then
    echo "${ZSH_VERSION}"
  else
    echo "Unknown"
  fi
}

# Args:
#   debug_log_file: string with a filename to save logs
#   use_validation_hit_server:
#          0 do not hit any Analytics server (for local tests)
#          1 use the Analytics validation Hit server (for integration tests)
#   config_file: string with a filename to save the config file. Defaults to
#          METRICS_CONFIG
function _enable_testing {
  _METRICS_DEBUG_LOG_FILE="$1"
  _METRICS_USE_VALIDATION_SERVER=$2
  if [[ $# -gt 2 ]]; then
    METRICS_CONFIG="$3"
  fi
  _METRICS_DEBUG=1
  METRICS_UUID="TEST"
}
