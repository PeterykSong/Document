import torch
import torchvision.transforms as T
import torchvision.datasets as datasets
import torchvision.models as models
import timm
from sklearn.manifold import TSNE
import matplotlib.pyplot as plt

# 1. 데이터셋 로드 및 전처리
transform = T.Compose([
    T.Resize(224),
    T.ToTensor(),
    T.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
])

cifar10 = datasets.CIFAR10(root='./data', train=False, download=True, transform=transform)
loader = torch.utils.data.DataLoader(cifar10, batch_size=256, shuffle=False, num_workers=4, pin_memory=True)

# 2. 모델 로딩
resnet = models.resnet18(pretrained=True)
resnet.eval()

vit = timm.create_model('vit_base_patch16_224', pretrained=True)
vit.eval()

# 3. Feature 추출 함수
def extract_features(model, dataloader, is_vit=False):
    features = []
    labels = []
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(device)
    model.to(device)

    with torch.no_grad():
        for images, lbls in dataloader:
            images = images.to(device)
            if is_vit:
                output = model.forward_features(images)  # ViT의 feature 추출
            else:
                # ResNet feature 추출
                x = model.conv1(images)
                x = model.bn1(x)
                x = model.relu(x)
                x = model.maxpool(x)
                x = model.layer1(x)
                x = model.layer2(x)
                x = model.layer3(x)
                x = model.layer4(x)
                x = model.avgpool(x)
                output = torch.flatten(x, 1)
            features.append(output.cpu())
            labels.append(lbls)

    return torch.cat(features), torch.cat(labels)

# 4. Feature 추출
resnet_features, resnet_labels = extract_features(resnet, loader, is_vit=False)
vit_features, vit_labels = extract_features(vit, loader, is_vit=True)

# 5. t-SNE 시각화
from sklearn.manifold import TSNE

def plot_tsne(features, labels, title):
    tsne = TSNE(n_components=2, perplexity=30, random_state=42)
    reduced = tsne.fit_transform(features)

    plt.figure(figsize=(10, 7))
    scatter = plt.scatter(reduced[:, 0], reduced[:, 1], c=labels, cmap='tab10', alpha=0.7)
    plt.colorbar(scatter, ticks=range(10))
    plt.title(title)
    plt.show()

# 6. 시각화 실행
plot_tsne(resnet_features.numpy(), resnet_labels.numpy(), 'ResNet18 Feature Embeddings (t-SNE)')
plot_tsne(vit_features.numpy(), vit_labels.numpy(), 'ViT Feature Embeddings (t-SNE)')
