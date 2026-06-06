#----------------------------------------------命令一 基础时空变阵指令---------------------------------------------------
# 正多边形编队变换，最大数量：>8
# 一字长蛇编队变换，最大数量：>8
#圆形编队变换，最大数量：>8
#隐形指令（散开），最大数量：>8
# test_cmd = "UVA1-6 以[3.4.5]为中心，变换成圆形编队，半径为1.5米，限时3秒"
#起始点为圆形，耗时为8s
# test_cmd = "现在的阵型太密集了，以UVA4为中心，横向拉开距离，变成一字长蛇阵，间距保持在 2 米，限时 3 秒完成"
# 10个UAV,起始点为圆形+矩形，耗时8s

#----------------------------------------------命令二 集群裂变与多目标调度----------------------------------------------------
# test_cmd = "队形一分为二。1到6号机以[3,4,5]为中心变成圆形；7号到10号机在圆形正上方2米形成正方形掩护阵型。两组动作必须在 5 秒内同步完成"
# 输出两任务 耗时 15s(起始点杂乱）,

#----------------------------------------------命令二 连续状态机时序指令----------------------------------------------------
#test_cmd = "准备谢幕表演：首先用 3 秒钟聚拢成一个以[2,2,10]为球形的最紧凑的球形编队并稳定悬停 2 秒钟后，然后全体向外发散，各自用 4 秒时间飞回初始起飞点降落"
#输出两任务（悬停+直接执行），球形为圆+上下两机，10个UAV耗时：起始点是一字排开为25s.
# 原来的指令:准备谢幕表演：首先用 3 秒钟聚拢成一个最紧凑的球形编队；稳定悬停 2 秒钟后；全体向外发散，各自用 4 秒时间飞回初始起飞点降落,ai通过断句会把悬停判给第二个任务







from openai import OpenAI
import re
import json
import time
import httpx

# -------------------------- 配置项（硬编码KEY） --------------------------
API_KEY = "sk-cp-9x3kcvFzbvnOjYlnl_pPis_CACTgZd0x7GKTmAU6YqjbDt951u5nTB7u__UvQjR7IdU0Ea5G2IZQydKfy6zbxGmhO7vg4EMwiGnPvr5KCv4sLoCtoGSPhuA"
BASE_URL = "https://api.minimax.chat/v1"
MODEL_NAME = "MiniMax-M2.7-highspeed"
# -------------------------------------------------------------------------

# ====================== 固定系统Prompt（新格式） ======================
SYSTEM_PROMPT = """
# 角色定位
你是无人机集群编队变换专属指令解析专家，唯一职责是将操作人员输入的自然语言指令，严格按照下方指定的JSON格式，解析为编队变换参数化指令，禁止处理任何与无人机编队任务无关的请求。

# 核心任务（大幅简化，减少思考时间）
1. 无遗漏、无篡改提取指令中的时间、编队类型、中心、半径/间距、触发条件等核心参数；
2. 严格按【固定CFR-JSON输出规范】生成内容，禁止修改字段名、字段顺序、枚举值；
3. 禁止编造信息，严格遵循预定义知识库；
4. 异常指令必须按规范返回错误码。

# 【编队类型与参数映射规则】
## 1. 标准图形（generation_mode = "parametric"）
- 指令出现「正三角形、正四边形、正多边形、圆形、球形、一字长蛇阵、直线」→ generation_mode固定为"parametric"
- parametric_data.formation_type 枚举值仅限：Triangle/Polygon/Sphere/Circle/Line/Lineup/Free
- parametric_data.formation_radius：当阵型为正多边形或者圆、球时为阵型外接圆/外接球半径；当为直线时为无人机间距


# 【trigger_condition 官方强制定义（零容忍）】
枚举值仅限以下3个，禁止使用任何其他值：
1. direct_execution：收到指令立即执行，单任务默认值
2. hover_and_wait：到达目标点后悬停等待，多任务终点或明确要求悬停时使用
3. continuous_transit：途经目标点不停车，多任务中间过渡时使用

# 2. 编队展开方向规则
- 指令出现「横向拉开、左右拉开」→ 一字阵型沿 X 轴水平排列，Y、Z坐标与编队中心保持一致
- 指令出现「纵向拉开、上下拉开」→ 一字阵型沿 Y 轴垂直排列，X、Z坐标与编队中心保持一致
- 无明确方向时，默认沿 X 轴横向展开

# 【字段精简规则】
1. iapf_safety_margin_factor：指令未提及修改该参数时，固定输出null，仅在明确要求修改时输出有效数值
2. motion_profile：指令未明确要求"柔和/快速"时，固定输出"normal"
3. constraints：固定输出默认值 ["minimal_topology_change", "no_trajectory_cross", "keep_safety_distance"]
4. wait_time：仅在trigger_condition为"hover_and_wait"且指令明确要求悬停时间时输出，否则固定输出None

# 固定CFR-JSON输出规范（零修改，严格遵循字段顺序）
【强制输出格式】无论单任务还是多任务，必须统一输出以下外层结构：
{
  "task_sequences": [
    {
      任务1内容
    },
    {
      任务2内容
    }
  ]
}

【单任务/子任务内部字段规范(严格遵循字段顺序)】
{
  "task_sequence_id": "数字，必填，任务序号，单任务=1，多任务递增",
  "duration_seconds": "浮点数，必填，编队变换完成总时长，单位：秒，指令未提及时用默认值3.0",
  "uav_id":"指令中指定无人机编号，格式[num1,num2,...],若无明确编号，则默认为从ROS中获得的所有UAV编号"
  "uav_count": "整数，必填，执行本次任务的无人机总数量，从指令中提取，若指令里未体现数量，则默认为从ROS中获得的所有UAV数量",
  "trigger_condition": "字符串，必填，枚举仅限：direct_execution/hover_and_wait/continuous_transit",
  "wait_time":"浮点数，必填,单位：秒",
  "iapf_safety_margin_factor": "必填，指令未提及时固定输出None",
  "motion_profile": "字符串，必填，运动轮廓约束，枚举值仅限：normal/smooth/aggressive；默认值为normal；smooth=柔和舒缓/无急加减速/放宽时间分配，aggressive=快速激进/尽快到位/收紧时间分配",
  "constraints":"数组，必填，指令提取的所有约束条件，默认必须包含[\"minimal_topology_change\", \"no_trajectory_cross\", \"keep_safety_distance\"]，可根据指令新增额外约束",
  "global_center": "数组，必填，编队全局中心3D坐标，格式[X,Y,Z]，指令未提及时用默认值[0.0, 0.0, 1.5]",
  "generation_mode": "字符串，必填，固定输出\"parametric\"",
  "parametric_data": "对象，必填，parametric模式下的编队参数",
{
  "formation_type": "字符串，必填，枚举仅限：Triangle/Polygon/Sphere/Circle/Line/Lineup/Free",
  "formation_radius": "浮点数，必填，机间间距/阵型半径，规范无人机形成图形的大小，单位：米，指令未提及时用默认值1.5",
}
}



# 输出铁则（零容忍）
1. 必须输出纯JSON，无任何解释、无markdown、无代码块、无多余符号；
2. 必须统一包裹在 { "task_sequences": [ ... ] } 内，单任务也必须放在数组里；
3. 禁止修改字段名、字段顺序、枚举值，仅可填充字段内容；
4. 禁止输出uav_allocations字段，禁止计算目标坐标；
5. 禁止违反精简规则，必须按要求输出默认值。

# 错误处理规则
1. 正常解析：error_code=0，error_msg=""
2. 信息缺失：未明确编队类型、未提供中心坐标，error_code=1
3. 实体不存在：使用了非法枚举值，error_code=2
4. 无关请求：与无人机编队任务无关，error_code=3
5. 解析失败：无法按规范解析，error_code=4
"""

# ====================== Few-Shot 示例（新格式） ======================
FEW_SHOT_EXAMPLES = """
【示例1：单任务标准图形】
用户指令：UAV1-6 以[3,4,5]为中心，变换成圆形编队，半径为1.5米，限时3秒
ROS信息：当前可用无人机编号: [1,2,3,4,5,6,7,8,9,10]，总数: 10
输出：
{
  "task_sequences": [
    {
      "task_sequence_id": 1,
      "duration_seconds": 3.0,
      "uav_id":[1,2,3,4,5,6]
      "uav_count": 6,
      "trigger_condition": "direct_execution",
      "wait_time":None,
      "iapf_safety_margin_factor": null,
      "motion_profile": "normal",
      "constraints": ["minimal_topology_change", "no_trajectory_cross", "keep_safety_distance"],
      "global_center": [3.0, 4.0, 5.0],
      "generation_mode": "parametric",
      "parametric_data": {
        "formation_type": "Circle",
        "formation_radius": 1.5
      },
    }
  ]
}

【示例2：多任务连续执行】
用户指令：准备谢幕表演：首先用 3 秒钟聚拢成一个以[2,2,10]为中心的球形编队，稳定悬停 2 秒钟后，全体向外发散，各自用 4 秒时间飞回初始起飞点
ROS信息：当前可用无人机编号: [1,2,3,4,5,6,7,8,9,10]，总数: 10
输出：
{
  "task_sequences": [
    {
      "task_sequence_id": 1,
      "duration_seconds": 3.0,
      "uav_id":[1,2,3,4,5,6,7,8,9,10],
      "uav_count": 10,
      "trigger_condition": "hover_and_wait",
      "wait_time":2.0,
      "iapf_safety_margin_factor": null,
      "motion_profile": "normal",
      "constraints": ["minimal_topology_change", "no_trajectory_cross", "keep_safety_distance"],
      "global_center": [2.0, 2.0, 10.0],
      "generation_mode": "parametric",
      "parametric_data": {
        "formation_type": "Sphere",
        "formation_radius": 1.5
      },
    },
    {
      "task_sequence_id": 2,
      "duration_seconds": 4.0,
      "uav_id":[1,2,3,4,5,6,7,8,9,10],
      "uav_count": 10,
      "trigger_condition": "direct_execution",
      "wait_time":None,
      "iapf_safety_margin_factor": null,
      "motion_profile": "normal",
      "constraints": ["minimal_topology_change", "no_trajectory_cross", "keep_safety_distance"],
      "global_center": [0.0, 0.0, 0.0],
      "generation_mode": "parametric",
      "parametric_data": {
        "formation_type": "Free",
        "formation_radius": 1.5
      },
    }
  ]
}

【示例3：一字长蛇阵（隐性指令）】
用户指令：现在的阵型太密集了，以UVA4为中心，横向拉开距离，变成一字长蛇阵，间距保持在 2 米，限时 3 秒完成
ROS信息：当前可用无人机编号: [1,2,3,4,5,6,7,8,9,10]，总数: 10
输出：
{
  "task_sequences": [
    {
      "task_sequence_id": 1,
      "duration_seconds": 3.0,
      "uav_id":[1,2,3,4,5,6,7,8,9,10],
      "uav_count": 10,
      "trigger_condition": "direct_execution",
      "wait_time":None,
      "iapf_safety_margin_factor": null,
      "motion_profile": "normal",
      "constraints": ["minimal_topology_change", "no_trajectory_cross", "keep_safety_distance"],
      "global_center": [1.0, 4.0, 5.0],
      "generation_mode": "parametric",
      "parametric_data": {
        "formation_type": "Line",
        "formation_radius": 2.0
      },
    }
  ]
}
"""


# ====================== 工具函数 ======================
def purify_json_content(raw_content: str) -> str:
    raw_content = re.sub(r"```json|```", "", raw_content).strip()
    match = re.search(r"\{.*\}", raw_content, re.DOTALL)
    return match.group(0) if match else raw_content


# ====================== 核心解析函数（新格式） ======================
def parse_uav_command(user_command: str, ros_aux_info: str = ""):
    full_prompt = (
            SYSTEM_PROMPT + "\n"
            + FEW_SHOT_EXAMPLES + "\n"
            + "【ROS实时情报】\n" + ros_aux_info + "\n"
            + "【用户指令】\n" + user_command + "\n"
            + "【输出】\n"
    )

    client = OpenAI(
    api_key=API_KEY, 
    base_url=BASE_URL,
    http_client=httpx.Client(trust_env=False)
    )

    max_retries = 3

    for attempt in range(max_retries):
        try:
            print(f"第{attempt + 1}次调用API解析指令...")
            response = client.chat.completions.create(
                model=MODEL_NAME,
                messages=[{"role": "user", "content": full_prompt}],
                temperature=0,
                top_p=0.01,
                max_tokens=4000,
                response_format={"type": "json_object"},
                timeout=60
            )

            raw_result = response.choices[0].message.content
            pure_json_str = purify_json_content(raw_result)
            cfr_result = json.loads(pure_json_str)

            # ====================== 新格式轻量级校验 ======================
            if "task_sequences" not in cfr_result or not isinstance(cfr_result["task_sequences"], list):
                raise ValueError("输出缺少task_sequences数组，外层结构错误")

            for task in cfr_result["task_sequences"]:
                # 校验必填字段
                required_fields = ["task_sequence_id", "duration_seconds", "trigger_condition",
                                   "iapf_safety_margin_factor", "motion_profile", "constraints",
                                   "global_center", "uav_count", "generation_mode",
                                   "parametric_data","uav_id","wait_time"]
                for field in required_fields:
                    if field not in task:
                        raise ValueError(f"子任务{task.get('task_sequence_id', '未知')}缺少必填字段：{field}")

                # 校验trigger_condition合法性
                legal_trigger = {"direct_execution", "hover_and_wait", "continuous_transit"}
                if task["trigger_condition"] not in legal_trigger:
                    raise ValueError(f"非法trigger_condition值：{task['trigger_condition']}")

                # 校验generation_mode
                if task["generation_mode"] != "parametric":
                    raise ValueError(f"本阶段仅支持parametric模式，实际输出：{task['generation_mode']}")

            print(" 解析结果校验通过！")
            return cfr_result

        except Exception as e:
            print(f" 第{attempt + 1}次解析失败：{str(e)}")
            if attempt == max_retries - 1:
                return {
                    "task_sequences": [],
                    "error_code": 4,
                    "error_msg": f"解析失败（重试{max_retries}次）：{str(e)}"
                }
            time.sleep(2)


# ====================== 测试 ======================
if __name__ == "__main__":
    test_cmd = "准备谢幕表演：首先用 3 秒钟聚拢成一个以[2,2,10]为球形的最紧凑的球形编队并稳定悬停 2 秒钟后，然后全体向外发散，各自用 4 秒时间飞回初始起飞点降落"
    test_ros = "当前可用无人机编号: [1,2,3,4,5,6,7,8,9,10]，总数: 10"
    result = parse_uav_command(test_cmd, test_ros)
    print("\n" + "=" * 50)
    print("最终解析结果：")
    print("=" * 50)
    print(json.dumps(result, indent=2, ensure_ascii=False))