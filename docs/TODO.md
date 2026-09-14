1. 标定 G1 前进、转向等基本数据
2. 阅读论文 SONIC 并尝试部署（稳定行走）
3. 实现 PoseNav：轻量版 ObjectNav，去除 graph.explore()，保留 Detectron2（但不允许覆盖目标）、goal graph 和 scene graph（异步），提供给上层（任务拆解编排）