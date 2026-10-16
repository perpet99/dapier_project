import os
import sys

# bot_modules가 있는 프로젝트 루트 경로를 sys.path에 추가 (어디서 실행하든 임포트 보장)
_cur_dir = os.path.dirname(os.path.abspath(__file__))
_root_dir = os.path.dirname(_cur_dir) if os.path.basename(_cur_dir) == 'scripts' else _cur_dir
for _p in [_root_dir, os.path.expanduser("~/ros2_amr_ws")]:
    if os.path.exists(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import discord
from discord.ext import commands

from bot_modules.config import DISCORD_TOKEN, GEMINI_API_KEY, get_waypoints_dict
from bot_modules.ai_agent import ask_gemini_intent
from bot_modules.ros2_runner import stop_robot_helper
from bot_modules.mission_queue import run_action_queue

# Discord Bot 초기화
intents = discord.Intents.default()
intents.message_content = True
bot = commands.Bot(command_prefix="!", intents=intents)


async def list_waypoints_helper(channel):
    """장소 목록 출력 헬퍼"""
    wps = get_waypoints_dict()
    embed = discord.Embed(
        title="📍 AMR 자율주행 등록 장소 목록",
        description="자연어로 편하게 말씀해주세요! (예: 'B지점 들렀다 A지점 가줘')",
        color=discord.Color.blue()
    )
    for wp_id, info in wps.items():
        embed.add_field(name=f"🔹 {wp_id}", value=f"{info['name']} (X:{info['x']}, Y:{info['y']})", inline=False)
    await channel.send(embed=embed)


@bot.event
async def on_ready():
    print("=" * 60)
    print(f"🤖 AMR 지능형 디스코드 봇 로그인 완료: {bot.user}")
    if GEMINI_API_KEY:
        print("🧠 Gemini LLM API 연동 활성화 (AI 자연어 판단 모드)")
    else:
        print("⚠️ GEMINI_API_KEY 미설정 (기존 키워드 매칭 모드 구동)")
    print("=" * 60)
    await bot.change_presence(activity=discord.Game(name="AI 자연어 주행 제어 대기 중!"))


@bot.event
async def on_message(message):
    """메시지 수신 시 AI 해석 및 키워드 Fallback 처리"""
    if message.author.bot:
        return

    raw_text = message.content.strip()
    text = raw_text.lower()

    # 1. 느낌표(!) 명령어 처리
    if text.startswith("!"):
        await bot.process_commands(message)
        return

    # 2. 최우선 긴급 정지 체크 (키워드 즉시 반응)
    if any(k in text for k in ["멈춰", "정지", "스톱", "stop", "서봐", "서라", "멈추", "그만"]):
        await stop_robot_helper(message.channel)
        return

    wps = get_waypoints_dict()

    # 3. Gemini LLM API 사용 시도
    if GEMINI_API_KEY:
        ai_plan = await ask_gemini_intent(raw_text, wps)
        if ai_plan and isinstance(ai_plan, dict):
            reply = ai_plan.get("reply", "")
            actions = ai_plan.get("actions", [])

            if reply:
                await message.channel.send(f"🤖 **[AI AMR 비서]** {reply}")

            if actions:
                await run_action_queue(message.channel, actions, wps)
            return

    # 4. Gemini 미사용 또는 실패 시 기존 키워드 매칭 (Fallback)
    if any(k in text for k in ["집", "홈", "home", "충전", "복귀", "돌아가", "집으로"]):
        await run_action_queue(message.channel, [{"type": "move", "target": "home"}], wps)
        return

    if any(k in text for k in ["순찰", "한 바퀴", "한바퀴", "patrol", "돌고 와"]):
        await run_action_queue(message.channel, [{"type": "patrol"}], wps)
        return

    if any(k in text for k in ["왕복", "스킬 테스트", "스킬테스트", "1초 직진", "1초 전진", "3초 직진", "3초 전진", "직진 테스트"]):
        await run_action_queue(message.channel, [{"type": "round_trip"}], wps)
        return

    if any(k in text for k in ["포인트 a", "지점 a", "작업대", "point a", "point_a", "a로"]):
        await run_action_queue(message.channel, [{"type": "move", "target": "point_a"}], wps)
        return

    if any(k in text for k in ["포인트 b", "지점 b", "배송", "point b", "point_b", "b로"]):
        await run_action_queue(message.channel, [{"type": "move", "target": "point_b"}], wps)
        return

    if any(k in text for k in ["포인트 c", "지점 c", "point c", "point_c", "c로"]):
        await run_action_queue(message.channel, [{"type": "move", "target": "point_c"}], wps)
        return

    if any(k in text for k in ["포인트 d", "지점 d", "point d", "point_d", "d로"]):
        await run_action_queue(message.channel, [{"type": "move", "target": "point_d"}], wps)
        return

    if any(k in text for k in ["목록", "장소", "어디", "위치"]):
        await list_waypoints_helper(message.channel)
        return

    await bot.process_commands(message)


# 기존 명령어 핸들러
@bot.command(name="목록", aliases=["list", "장소"])
async def cmd_list(ctx):
    await list_waypoints_helper(ctx.channel)

@bot.command(name="이동", aliases=["go", "move"])
async def cmd_move(ctx, wp_id: str):
    wps = get_waypoints_dict()
    await run_action_queue(ctx.channel, [{"type": "move", "target": wp_id}], wps)

@bot.command(name="홈", aliases=["집", "home"])
async def cmd_home(ctx):
    wps = get_waypoints_dict()
    await run_action_queue(ctx.channel, [{"type": "move", "target": "home"}], wps)

@bot.command(name="순찰", aliases=["patrol"])
async def cmd_patrol(ctx):
    wps = get_waypoints_dict()
    await run_action_queue(ctx.channel, [{"type": "patrol"}], wps)

@bot.command(name="왕복", aliases=["test", "round_trip"])
async def cmd_round_trip(ctx):
    wps = get_waypoints_dict()
    await run_action_queue(ctx.channel, [{"type": "round_trip"}], wps)

@bot.command(name="정지", aliases=["stop"])
async def cmd_stop(ctx):
    await stop_robot_helper(ctx.channel)


def main():
    if DISCORD_TOKEN == "YOUR_BOT_TOKEN_HERE":
        print("❌ DISCORD_BOT_TOKEN이 설정되지 않았습니다!")
        return
    bot.run(DISCORD_TOKEN)


if __name__ == '__main__':
    main()
