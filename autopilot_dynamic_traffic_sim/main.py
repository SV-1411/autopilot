import pygame

from core.environment import Environment
from config import FPS, SCREEN_HEIGHT, SCREEN_WIDTH


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Autopilot Dynamic Traffic Simulation")
    clock = pygame.time.Clock()

    env = Environment()

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            else:
                env.handle_event(event)

        env.update(dt)

        screen.fill((0, 0, 0))
        env.render(screen)
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
