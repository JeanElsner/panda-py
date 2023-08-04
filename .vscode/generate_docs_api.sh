package_name="panda-python"

if metadata=$(pip show "$package_name" 2>/dev/null); then
    if [[ $metadata == *"Editable"* ]]; then
        echo -e "\e[0;31mPackage '$package_name' is installed in editable mode. \e[0m"
        exit 1
    else
        location=$(pip show -f "$package_name" | awk '/^Location:/ {print $2}')
        sphinx-apidoc -e -d 1 -M -T -f -o docs $location/panda_python
    fi
else
    echo -e "\e[0;31mPackage '$package_name' is not installed. \e[0m"
    exit 1
fi
