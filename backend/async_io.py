"""
Testing async io for input and output.
Not used in the project. Can be deleted.
"""

import asyncio
import aioconsole

class AsyncIO:
    def __init__(self):
        pass
        self.loop = asyncio.get_event_loop()

    async def async_input(self, prompt):
        while True:
            try:
                a = await aioconsole.ainput(prompt)
                print(a)
            except Exception as e:
                print(e)
                continue
