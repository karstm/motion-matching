import sys

def process_arguments(args):
    input_file = args[0]
    arguments = args[1:]

    with open(input_file, 'r') as f:
        contents = f.readlines()

    frames_line = None
    for i, line in enumerate(contents):
        if line.startswith('Frames: '):
            frames_line = i
            break

    if frames_line is None:
        print('Error: Frames line not found in the input file.')
        return

    for i in range(0, len(arguments), 3):
        appendix = arguments[i]
        start = int(arguments[i + 1])
        end = int(arguments[i + 2])

        output_file = input_file.rsplit('.', 1)[0] + '_' + appendix + '.bvh'

        with open(output_file, 'w') as f:
            num_frames = end - start

            for j in range(frames_line):
                f.write(contents[j])

            f.write(f'Frames: {num_frames}\n')
            f.write(contents[frames_line + 1])

            next_line = frames_line + 2 + start
            for _ in range(num_frames):
                f.write(contents[next_line])
                next_line += 1

        print(f'Successfully created {output_file}.')

if __name__ == '__main__':
    if len(sys.argv) < 5 or len(sys.argv[2:]) % 3 != 0:
        print('Error: Invalid number of arguments.')
    else:
        process_arguments(sys.argv[1:])