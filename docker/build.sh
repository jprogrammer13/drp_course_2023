docker build -t drp_agilex_limo:latest .

# create local home for drp

if [ -d "${HOME}/drp_home" ]; then
  echo "Directory drp_home exists."
else
    echo "Creating drp_home dir."
    mkdir -p ${HOME}/drp_home
fi