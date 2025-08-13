function isInCollision = checkSelfCollision(robot, q)

    isInCollision = checkCollision(robot, q, 'SkippedSelfCollisions', 'parent');

end