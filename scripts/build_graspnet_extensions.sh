#!/usr/bin/env bash
set -euo pipefail

VENV_PATH="${VENV_PATH:-/home/grga/Documents/Depth-Anything-3/.venv}"
GRASPNET_ROOT="${GRASPNET_ROOT:-/home/grga/Documents/graspnet-baseline}"
REAL_CUDA_HOME="${REAL_CUDA_HOME:-/opt/cuda}"
CUDA_COMPAT_HOME="${CUDA_COMPAT_HOME:-${HOME}/.cache/so101-cuda-compat-12.8}"
TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-8.6}"

if [[ ! -x "${VENV_PATH}/bin/python" ]]; then
    echo "Python venv not found: ${VENV_PATH}" >&2
    exit 1
fi

if [[ ! -x "${REAL_CUDA_HOME}/bin/nvcc" ]]; then
    echo "nvcc not found under ${REAL_CUDA_HOME}" >&2
    exit 1
fi

if [[ ! -d "${GRASPNET_ROOT}/pointnet2" || ! -d "${GRASPNET_ROOT}/knn" ]]; then
    echo "graspnet-baseline checkout not found under ${GRASPNET_ROOT}" >&2
    exit 1
fi

mkdir -p "${CUDA_COMPAT_HOME}/bin"
rm -rf "${CUDA_COMPAT_HOME}/include" "${CUDA_COMPAT_HOME}/lib64"
ln -s "${REAL_CUDA_HOME}/include" "${CUDA_COMPAT_HOME}/include"
ln -s "${REAL_CUDA_HOME}/lib64" "${CUDA_COMPAT_HOME}/lib64"

NVCC_WRAPPER="${CUDA_COMPAT_HOME}/bin/nvcc"
printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -euo pipefail' \
    'if [[ "${1:-}" == "--version" ]]; then' \
    '    echo "nvcc: NVIDIA (R) Cuda compiler driver"' \
    '    echo "Cuda compilation tools, release 12.8, V12.8.93"' \
    '    exit 0' \
    'fi' \
    'exec /opt/cuda/bin/nvcc "$@"' > "${NVCC_WRAPPER}"
chmod +x "${NVCC_WRAPPER}"

export CUDA_HOME="${CUDA_COMPAT_HOME}"
export PATH="${CUDA_HOME}/bin:${PATH}"
export FORCE_CUDA=1
export TORCH_CUDA_ARCH_LIST

pushd "${GRASPNET_ROOT}/pointnet2" >/dev/null
"${VENV_PATH}/bin/python" setup.py install
popd >/dev/null

pushd "${GRASPNET_ROOT}/knn" >/dev/null
"${VENV_PATH}/bin/python" setup.py install
popd >/dev/null

echo "GraspNet extensions built with CUDA_HOME=${CUDA_HOME}"
