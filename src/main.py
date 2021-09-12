from loop import Loop
import argparse
import logging

logging.basicConfig(level=logging.INFO)

# Argumentos
parser = argparse.ArgumentParser(description='ALP-Winners system')
parser.add_argument('--team-color', dest='team_color', type=str, choices=['yellow', 'blue'], required=True, help='Team color.')
parser.add_argument('--team-side', dest='team_side', type=str, choices=['left', 'right'], required=True, help='Team side.')
parser.add_argument('--immediate-start', dest='immediate_start', action='store_const', const=True, default=False, help='If robots should start moving without VSSReferee telling so.')
args = parser.parse_args()

# Instancia o programa principal
loop = Loop(
    draw_uvf=False, 
    team_yellow=True if args.team_color == 'yellow' else False,
    team_side=1 if args.team_side == 'left' else -1,
    immediate_start=args.immediate_start
)

loop.run()