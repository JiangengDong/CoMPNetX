import argparse
from genericpath import exists
import os

import torch
from torch.nn import functional as F
from torch.utils.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm, trange
import yaml
import datetime

from data import CoMPNetXDataset
from models import EnetConstraint, PNet, VoxelEncoder


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--env', type=str, choices=("kitchen", "bartender"), default="bartender", help="which environment to train")
    parser.add_argument('--use_text', action="store_true", help="use text embedding or NTP embedding for constraint representation")
    parser.add_argument('--use_reach', action="store_true", help="add reach path to training dataset")
    parser.add_argument('--use_tsr', action="store_true", help="predict virtual TSR config along with manipulator config")

    # Model parameters
    parser.add_argument('--outsz_enet', type=int, default=256, help='dimension of ENets output vector')
    parser.add_argument('--outsz_constraint', type=int, default=128, help='dimension of ENetConstaint output vector')

    # training parameters
    parser.add_argument('--num_epochs', type=int, default=400)
    parser.add_argument('--checkpoint_step', type=int, default=10, help='step size for saving trained models')
    parser.add_argument('--batch_size', type=int, default=256)
    parser.add_argument('--lr', type=float, default=0.0001)

    # utils
    parser.add_argument('--output_dir', type=str, default='./data/experiments/', help='where to save all the output files')
    parser.add_argument('--use_cuda', action="store_true", help="use CUDA for faster training")
    parser.add_argument('--overwrite', action="store_true", help="ignore existing files in the output directory")

    return parser.parse_args()


def process_args(args: argparse.Namespace) -> argparse.Namespace:
    args.insz_enet = 32 if args.env == "kitchen" else 33
    args.insz_constraint = 4096 if args.use_text else 270
    args.outsz_pnet = 13 if args.use_tsr else 7
    args.insz_pnet = args.outsz_pnet * 2 + args.outsz_enet + args.outsz_constraint
    args.device = "cuda" if torch.cuda.is_available() and args.use_cuda else "cpu"
    args.training_time = datetime.datetime.now()

    if os.path.exists(args.output_dir) and not args.overwrite:
        overwrite = input("{} already exists. Overwrite? [y/N]".format(args.output_dir))
        if overwrite != "y":
            print("\033[91mNot overwriting - exiting. \033[0m")
            exit()

    args.tensorboard_dir = os.path.join(args.output_dir, "tensorboard")
    args.weight_dir = os.path.join(args.output_dir, "model_weight")
    args.torchscript_dir = os.path.join(args.output_dir, "torchscript")
    args.embedding_dir = os.path.join(args.output_dir, "embedding")

    return args


def prepare_directories(args):
    os.makedirs(args.output_dir, exist_ok=True)
    os.makedirs(args.tensorboard_dir, exist_ok=True)
    os.makedirs(args.weight_dir, exist_ok=True)
    os.makedirs(args.torchscript_dir, exist_ok=True)
    os.makedirs(args.embedding_dir, exist_ok=True)
    with open(os.path.join(args.output_dir, "args.yaml"), "w") as f:
        yaml.dump(args.__dict__, f)


def main(args):
    # models
    enet = VoxelEncoder(args.insz_enet, args.outsz_enet).to(args.device)
    enet_constraint = EnetConstraint(args.insz_constraint, args.outsz_constraint).to(args.device)
    pnet = PNet(args.insz_pnet, args.outsz_pnet).to(args.device)
    # dataset
    dataset = CoMPNetXDataset(env=args.env, use_text=args.use_text, use_reach=args.use_reach, use_tsr_config=args.use_tsr, use_manifold_distance=False)
    dataloader = DataLoader(dataset, args.batch_size, shuffle=True, drop_last=True)
    # optimizer
    optimizer = torch.optim.Adagrad(list(enet.parameters())+list(enet_constraint.parameters())+list(pnet.parameters()))
    # utils
    writer = SummaryWriter(args.tensorboard_dir)

    # Train the models
    for epoch in trange(args.num_epochs, desc="Epoch"):
        for (inputs, outputs, voxels, task_embeddings) in tqdm(dataloader, leave=False):
            inputs = inputs.to(args.device)
            outputs = outputs.to(args.device)
            voxels = voxels.to(args.device)
            task_embeddings = task_embeddings.to(args.device)
            # forward
            Z_c = enet_constraint.forward(task_embeddings)
            Z_o = enet.forward(voxels)
            predicts = pnet.forward(torch.cat((Z_o, Z_c, inputs), 1))
            # backward
            loss = F.mse_loss(predicts, outputs)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
        writer.add_scalar("loss", loss, epoch)
        # Save the models
        if epoch % args.checkpoint_step == 0:
            torch.save(enet.state_dict(), os.path.join(args.weight_dir, "enet_{}.pkl".format(epoch)))
            torch.save(pnet.state_dict(), os.path.join(args.weight_dir, "pnet_{}.pkl".format(epoch)))
            torch.save(enet_constraint.state_dict(), os.path.join(args.weight_dir, "enet_constraint_{}.pkl".format(epoch)))

    writer.close()

    # TODO: produce torchscript and embeddings


if __name__ == '__main__':
    args = process_args(get_args())
    prepare_directories(args)
    main(args)
