(define (problem actionmove-test)
    (:domain robocup-work-transport)

    (:objects
        robot1 - robot
        slot1 slot2 slot3 - slot
        start loc1 loc3 - location
        cube1 cube2 cube3 - object
        p1l1 p2l1 p3l1 - position
    )

    (:init
        (at-robot robot1 start)
        (slot-free robot1 slot1)
        (slot-free robot1 slot2)
        (slot-free robot1 slot3)
        (gripper-free robot1)
        (obj-at cube1 loc3)
        (obj-at cube2 loc3)
        (obj-at cube3 loc3)
        (position-free loc1 p1l1)
        (position-free loc1 p2l1)
        (position-free loc1 p3l1)
        (connected start loc3)
        (connected loc3 loc1)droid@DROID:~/WORK2-2026 $ git rebase --abort
droid@DROID:~/WORK2-2026 $ git add build/planning/colcon_command_prefix_setup_py.sh.env
The following paths are ignored by one of your .gitignore files:
build
hint: Use -f if you really want to add them.
hint: Disable this message with "git config advice.addIgnoredFile false"
    )

    (:goal
        (and
            (obj-at cube3 loc1)
            (obj-at cube2 loc1)
            (obj-at cube1 loc1)   
        )
    )

)