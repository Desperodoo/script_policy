# RoboTwin Environment Setup

当前仓库使用独立 conda 环境：
- `script_policy`

## 1. 创建并安装基础依赖

```bash
bash scripts/setup_robotwin_env.sh
```

这会完成：
- 创建 `python=3.10` 的 conda 环境
- 安装 `third_party/RoboTwin/script/requirements.txt`
- 安装 `setuptools<81`
- 安装 `pytorch3d`
- 克隆并安装 `curobo`
- 对 `sapien` 和 `mplib` 打 RoboTwin 官方兼容补丁

## 2. 下载 RoboTwin 资产

```bash
bash scripts/download_robotwin_assets.sh
```

这会下载并解压：
- `background_texture`
- `embodiments`
- `objects`

并执行：
- `third_party/RoboTwin/script/update_embodiment_config_path.py`

## 3. 已知注意事项

- `sapien 3.0.0b1` 依赖 `pkg_resources`，因此环境里需要 `setuptools<81`
- RoboTwin 根目录本身不是标准 Python 包，不需要直接 `pip install -e third_party/RoboTwin`
- 如果缺少资产，导入 `envs` 时会报 `assets/objects/objaverse/list.json` 找不到
