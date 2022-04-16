PYDIR=$(python3 -c "import sys;print('python{}.{}'.format(sys.version_info.major,sys.version_info.minor))")
export PYTHONPATH=$PYTHONPATH:/usr/lib/llvm-${llvm_ver}/lib/${PYDIR}/site-packages
export PYTHONPATH=$(echo $PYTHONPATH | tr ':' '\n' | uniq | tr '\n' ':')
export PYTHONPATH=${PYTHONPATH%:}
