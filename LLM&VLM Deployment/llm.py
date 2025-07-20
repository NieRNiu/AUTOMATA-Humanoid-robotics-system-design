import asyncio
from volcenginesdkarkruntime import AsyncArk

from prompt import SYSTEM_PROMPT
import api

client = AsyncArk(api_key=api.ARK_API_KEY)

async def main() -> None:
    stream = await client.chat.completions.create(
        model=api.LLM_MODEL,
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": "带我去厕所"},
        ],
        stream=True
    )
    async for completion in stream:
        print(completion.choices[0].delta.content, end="")
    print()

asyncio.run(main())