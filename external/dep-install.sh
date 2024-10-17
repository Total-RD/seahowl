#!/bin/bash

# Initialize options
OPTION_DEPENDENCY=""
OPTION_KEEP_SRC_AS_IS_FLAG=false
OPTION_FORCE_SRC_CLONE_FLAG=false
OPTION_SRC_SHALLOW_CLONE_FLAG=false
OPTION_SKIP_BUILD_FLAG=false
OPTION_MAKE_JOBS_NUMBER=4

# Initialize data
DEPENDENCY_COUNT=0
declare -A DEPENDENCY_MAP
declare -A DEPENDENCY_DATA
declare -A EXEC_DATA


# Initialize env variables
SCRIPT_FOLDER="$(dirname "$(realpath "$0")")"

# Constants
STATUS_SUCCESS="> done"
STATUS_ERROR="x ERROR !"
STATUS_SKIPPED="- skipped"
STATUS_NONE="-"

# #############################################################################
# #############################################################################
# FUNCTIONS
# #############################################################################
# #############################################################################

# #############################################################################
# UTILS

log_start() { echo -e "\n\033[1;35m======== $1 ========\033[0m"; }
log_sub() { echo -e "\e[1;33m$1\033[0m"; }
log_warn() { echo -e "\033[0;33m$1\033[0m"; }
log_error() { echo -e "\033[0;31m$1\033[0m"; }
log_kv() { echo -e "\033[1;32m$1\033[0m$2";}

fatal() { log_error "$1";  exit 1 ; }

display_table() {
    local headers=("$1")
    local data=("${@:2}")
    # local prefix=$3

    # split
    IFS=";" read -r -a headers <<< "$headers"

    # Calulate col size
    declare -A col_width
    for header in "${headers[@]}"; do
        col_width[$header]=${#header}
        # $(echo -n "$str" | wc -m)  # Count characters (not bytes)
    done


    for row in "${data[@]}"; do
        IFS=";" read -r -a row_data <<< "$row"
        for i in "${!row_data[@]}"; do
            header=${headers[$i]}
            [[ ${#row_data[$i]} -gt ${col_width[$header]} ]] && col_width[$header]=${#row_data[$i]}
        done
    done

    for header in "${headers[@]}"; do
        col_width[$header]=$(( col_width[$header] + 5 ))
    done    

    # Display Headers
    for header in "${headers[@]}"; do
        # printf "\033[0;33m%-*s\033[0m " "${col_width[$header]}" "$header"
        printf "%-*s " "${col_width[$header]}" "$header"
    done
    echo

    # Display Header separator
    for header in "${headers[@]}"; do
        printf "%-*s " "${col_width[$header]}" "$(printf '%*s' "${col_width[$header]}" | tr ' ' '-')"
    done
    echo

    # Display Data
    for row in "${data[@]}"; do
        IFS=";" read -r -a row_data <<< "$row"
        for i in "${!row_data[@]}"; do
            header=${headers[$i]}
            val="${row_data[$i]}"
            printf -v padded "%-*s " "${col_width[$header]}" "$val"
            case ${val:0:2} in
                '> ') c="\033[0;32m" ;;
                '- ') c="\033[3;97m" ;;
                'x ') c="\033[0;31m" ;;
                *)    c="" ;;
            esac
            printf "$c%s\033[0m" "$padded"
        done
        echo
    done
}

# #############################################################################
# OPTIONS MANAGEMENT

show_help() {
  log_start "Usage"
  echo -e "Usage: \033[0;33m$0\033[0m \033[0;32m[options]\033[0m"
  echo "  1. Define a list of candidate dependency libs."
  echo "  2. For each dependency, get the remote lib sources, override, build and install."
  log_start "Options"
  log_kv "  -d,  --dependency <str>" "           Specify a valid dependency (with dep-install-<str>.cgf file)⁾ to install, if not provided, all dependencies will be installed"
  log_kv "  -k,  --keep-src-as-is" "             Keep sources as is (default: false)"
  log_kv "  -c,  --force-src-clone" "            Force clone dependency repository, and remove old sources if exist (default: false)"
  log_kv "  -sc, --src-shallow-clone" "          When cloning, use the shallow clone option (--depth 1), could be incompatible with checkout (default: false)"
  log_kv "  -s,  --skip-build" "                 Skip the build step (default: false)"
  log_kv "  -j,  --make-jobs-number <num>" "     Define the numbers of jobs to use in make (default: 4)"
  log_kv "  -h,  --help" "                       Show this help message"
  echo
}

options_parse() {

    while [[ "$#" -gt 0 ]]; do
    case $1 in
        -h|--help) show_help; exit 0 ;;
        -k|--keep-src-as-is) OPTION_KEEP_SRC_AS_IS_FLAG=true ;;
        -c|--force-src-clone) OPTION_FORCE_SRC_CLONE_FLAG=true ;;
        -s|--skip-build) OPTION_SKIP_BUILD_FLAG=true ;;
        -sc|--src-shallow-clone) OPTION_SRC_SHALLOW_CLONE_FLAG=true ;;
        -j|--make-jobs-number) 
        if [[ "$2" =~ ^[0-9]+$ ]]; then
            OPTION_MAKE_JOBS_NUMBER=$2
            shift
        else
            echo "Error: --make-jobs-number requires a valid integer argument."
            exit 1
        fi
        ;;
        -d|--dependency)
        if [[ -n "$2" ]]; then
            OPTION_DEPENDENCY=$2
            shift
        else
            echo "Error: --dependency requires a valid string argument."
            exit 1
        fi
        ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
    done


    log_start "Options"
    log_kv "  dependency" "=$OPTION_DEPENDENCY"
    log_kv "  keep_src_as_is" "=$OPTION_KEEP_SRC_AS_IS_FLAG"
    log_kv "  force_src_clone" "=$OPTION_FORCE_SRC_CLONE_FLAG"
    log_kv "  skip_build" "=$OPTION_SKIP_BUILD_FLAG"
    log_kv "  make_jobs_number" "=$OPTION_MAKE_JOBS_NUMBER"

}

# #############################################################################
# CONTEXT

context_info() {
    log_start "Context"
    log_kv "  script folder" "=$SCRIPT_FOLDER"
}

# #############################################################################
# DEPENDENCIES MANAGEMENT

get_dependencies() {

    log_start "Gathering dependency configurations"
    
    local config_file_list=()
    if [ "$OPTION_DEPENDENCY" = "" ]; then
        echo "  No dependency option provided"
        log_sub "Search for dependency configuration files ..."

        # search config files    
        local pattern="dep-install-*.cfg"
        config_file_list=$(ls $SCRIPT_FOLDER/$pattern 2>/dev/null)
        # Check if the command was not successful or no files found
        if [ $? -ne 0 ]; then
            fatal "Error when searching configurations files or no configuration files found."
        fi

        # TODO
        echo "  Found $(echo "$config_file_list" | wc -l)"

    else
        echo "  Dependency option provided $OPTION_DEPENDENCY"
        local config_file="$SCRIPT_FOLDER/dep-install-$OPTION_DEPENDENCY.cfg"
        if [ ! -e "$config_file" ]; then
            fatal "Error $config_file not found."
        fi
        config_file_list=($config_file)
    fi

    log_sub "Load configuration files ..."
    for cfg_filepath in $config_file_list; do
        echo "  Processing file: $cfg_filepath"
        # load config file
        local NAME GIT_URL GIT_TAG OPTION
        # local 
        source "$cfg_filepath"
        DEPENDENCY_DATA["$NAME/NAME"]=$NAME
        DEPENDENCY_DATA["$NAME/ORDER"]=$ORDER
        DEPENDENCY_DATA["$NAME/GIT_URL"]=$GIT_URL
        DEPENDENCY_DATA["$NAME/GIT_TAG"]=$GIT_TAG
        DEPENDENCY_DATA["$NAME/OPTION"]=$OPTION
        DEPENDENCY_MAP["$ORDER"]=$NAME
        ((DEPENDENCY_COUNT++))
    done

    echo
    log_kv "  List ordered ($DEPENDENCY_COUNT) "
    sorted_keys=($(for key in "${!DEPENDENCY_MAP[@]}"; do echo "$key"; done | sort))    
    for key in "${sorted_keys[@]}"; do
        echo "  ${DEPENDENCY_MAP[$key]}"
    done

}

dependency_process() {
    local dependency=$1

    local exec_starttime=$(date +%s)

    local GIT_URL=${DEPENDENCY_DATA["$dependency/GIT_URL"]}
    local GIT_TAG=${DEPENDENCY_DATA["$dependency/GIT_TAG"]}
    local OPTION=${DEPENDENCY_DATA["$dependency/OPTION"]}

    log_start "PROCESS DEPENDENCY: $dependency"

    log_kv "  GIT_URL=" "$GIT_URL"
    log_kv "  GIT_TAG=" "$GIT_TAG"
    log_kv "  OPTION=" "$OPTION"
    echo

    dependency_process_src "$GIT_URL" "$GIT_TAG" "$dependency"
    dependency_process_override "$dependency"
    dependency_process_compile_install "$dependency" "$OPTION"

    local exec_endtime=$(date +%s)
    local exec_time=$((exec_endtime - exec_starttime))
    
    echo
    log_sub "Info"
    log_kv "  process time=" "$exec_time seconds"
    
    EXEC_DATA["$dependency/duration/label"]="$exec_time s"

}
 
dependency_process_src() {
    local git_url=$1
    local git_tag=$2
    local dir_name=$3


    if [ "$OPTION_KEEP_SRC_AS_IS_FLAG" = true ]; then

        echo "  option --keep-if-exists-and-build true ..."
        echo "  Keep the folder as is ..."
        echo

        EXEC_DATA["$dependency/src/label"]="- skipped"

    else

        if [ "$OPTION_FORCE_SRC_CLONE_FLAG" = true ]; then

            echo "  option --force-src-clone true ..."
            log_sub "Remove folder if exists ..."
            rm -rf "$dir_name"
            echo "  done"
            dependency_process_src_clone "$git_url" "$git_tag" "$dir_name"

        else
            log_sub "Check if folder exists ..."
            
            if [ -d "$dir_name" ]; then

                echo "  $dir_name folder exists ..."
                dependency_process_src_checkout "$git_url" "$git_tag" "$dir_name"

            else

                echo "  $dir_name folder does not exist ..."
                dependency_process_src_clone "$git_url" "$git_tag" "$dir_name"

            fi
        fi
    fi
}
 
dependency_process_src_clone() {
    local git_url=$1
    local git_tag=$2
    local dependency=$3
    
    local dir_name=$dependency

    log_sub "Cloning ..."

    local exec_status=$STATUS_ERROR
    local exec_starttime=$(date +%s)

    # Clone
    if [ "$OPTION_SRC_SHALLOW_CLONE_FLAG" = true ]; then
        echo "  Shallow clone ..."
        if ( git clone --depth 1 --branch "$git_tag" "$git_url" "$dir_name") then
            exec_status=$STATUS_SUCCESS
        fi        
    else
        if ( git clone --branch "$git_tag" "$git_url" "$dir_name") then
            exec_status=$STATUS_SUCCESS
        fi
    fi


    local exec_endtime=$(date +%s)
    local exec_time=$((exec_endtime - exec_starttime))
    log_kv "  cloning time=" "$exec_time seconds"
    
    EXEC_DATA["$dependency/src/label"]="$exec_status (Clone) ($exec_time s)"
}

dependency_process_src_checkout() {
    local git_url=$1
    local git_tag=$2
    local dir_name=$3

    log_sub "Checkout ..."

    local exec_status=$STATUS_ERROR
    local exec_starttime=$(date +%s)


    cd "$dir_name"
    # TODO INPROGRESS
    if (
        current_ref=$(git rev-parse --abbrev-ref HEAD | sed 's/^[ \t]*//;s/[ \t]*$//') &&
        log_kv "  Current ref: " "$current_ref" &&
        log_kv "  Target  ref: " "$git_tag" 
        
    ) then
        if (
            # does not work with clone shallow
            echo "  Checkout" &&
            git fetch origin "$git_tag" &&
            git checkout -B "$git_tag" "origin/$git_tag"
        ) then
            exec_status=$STATUS_SUCCESS
        fi
    fi    
    cd ..

    local exec_endtime=$(date +%s)
    local exec_time=$((exec_endtime - exec_starttime))
    log_kv "  checkout time=" "$exec_time seconds"

    EXEC_DATA["$dependency/src/label"]="$exec_status (checkout) ($exec_time s)"
}

dependency_process_override() {
    local dependency=$1

    log_sub "Overriding CMakeLists ..."

    local filename="CMakeLists.$dependency"
    local file="$SCRIPT_FOLDER/$filename"

    local exec_status=$STATUS_ERROR

    if [ -e "$file" ]; then
        echo "  File $filename exists."
        echo "  Overriding..."
        if (cp "$file" "$dependency/CMakeLists.txt") then
            exec_status=$STATUS_SUCCESS
        fi
    else
        echo "  No file, do nothing"
        exec_status="-"
    fi

    EXEC_DATA["$dependency/override/label"]=$exec_status

}

dependency_process_compile_install() {
    local dir_name=$1
    local options=$2
    local build_dir="${dir_name}/build"

    local exec_status=$STATUS_ERROR
    local exec_starttime=$(date +%s)

    log_sub "Building ..."

    if [ "$OPTION_SKIP_BUILD_FLAG" = true ]; then
        echo "  option --skip-build true ..."
        echo "  Skip the build step ..."
        exec_status="- skipped"
    else

        if (
            # Remove and recreate build folder
            echo "  create build dir ..." &&
            rm -rf "$build_dir" &&
            mkdir -p "$build_dir" &&

            # Build
            cd "$build_dir" &&
            echo "  cmake ..."  && 
            cmake  -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles"  $OPTION -DCMAKE_INSTALL_PREFIX=../../../install .. &&
            echo "  make ..." &&
            make -j $OPTION_MAKE_JOBS_NUMBER &&
            echo "  install ..." &&
            make install &&
            cd ../..

        ) then
            exec_status=$STATUS_SUCCESS
        fi
    fi

    local exec_endtime=$(date +%s)
    local exec_time=$((exec_endtime - exec_starttime))
    log_kv "  build time=" "$exec_time seconds"

    EXEC_DATA["$dependency/build/label"]="$exec_status ($exec_time s)"

}

# #############################################################################
# #############################################################################
# MAIN
# #############################################################################
# #############################################################################


options_parse $1 $2 $3 $4 $5 $6 $7 $8 $9
context_info
get_dependencies
# dependency_process
summary_data=()
sorted_keys=($(for key in "${!DEPENDENCY_MAP[@]}"; do echo "$key"; done | sort))    
for key in "${sorted_keys[@]}"; do
    dependency="${DEPENDENCY_MAP[$key]}"
    dependency_process $dependency
    summary_data+=("${DEPENDENCY_MAP[$key]};${EXEC_DATA["$dependency/duration/label"]};${EXEC_DATA["$dependency/src/label"]};${EXEC_DATA["$dependency/override/label"]};${EXEC_DATA["$dependency/build/label"]}")
done

# display summary
echo
log_start "Summary"
headers=("Dependency;Duration;Get Src;Override;Build/Install")
display_table "$headers" "${summary_data[@]}" "  "

echo








